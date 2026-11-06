/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#include "agc.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdint.h>

#include <libcamera/base/log.h>

#include "control_ids.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftExposure)

using namespace std::literals::chrono_literals;

namespace ipa::soft::algorithms {

/*
 * The number of bins to use for the optimal exposure calculations.
 */
static constexpr unsigned int kExposureBinsCount = 5;

/*
 * The exposure is optimal when the mean sample value of the histogram is
 * in the middle of the range.
 */
static constexpr float kExposureOptimal = kExposureBinsCount / 2.0;

/*
 * This implements the hysteresis for the exposure adjustment.
 * It is small enough to have the exposure close to the optimal, and is big
 * enough to prevent the exposure from wobbling around the optimal value.
 */
static constexpr float kExposureSatisfactory = 0.2;

/*
 * Highlight protection, defaults of the tuning file keys: above
 * saturatedMax of burnt pixels the exposure goes down, as long as the MSV
 * stays above msvMin; below half of saturatedMax it may go up again. The
 * gap is the hysteresis.
 */
static constexpr double kSaturatedMaxDefault = 0.04;
static constexpr float kExposureMSVMinDefault = 1.6;

/*
 * Frames to wait at most for a change to show in the sensor controls
 * reported for a frame, before deciding again anyway.
 */
static constexpr unsigned int kMaxWait = 8;

/*
 * Lamps on 50 Hz mains flicker at 100 Hz: an exposure that is not a
 * multiple of 10 ms shows it as bands rolling through the image.
 */
static constexpr unsigned int kFlickerPeriodDefault = 10000;

/* The shortest frame the frame duration limits offer, in microseconds. */
static constexpr int64_t kMinFrameTime = 33333;

/* The linear mean the AGC settles on, until it has settled once. */
static constexpr double kYGoalDefault = 0.25;

Agc::Agc()
	: pending_(false), exposureReq_(0), gainReq_(0), waited_(0),
	  target_(kExposureOptimal), msvMin_(kExposureMSVMinDefault),
	  saturatedMax_(kSaturatedMaxDefault), saturatedMin_(kSaturatedMaxDefault / 2)
{
}

int Agc::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	target_ = tuningData["target"].get<float>(kExposureOptimal);
	msvMin_ = tuningData["msvMin"].get<float>(kExposureMSVMinDefault);
	saturatedMax_ = tuningData["saturatedMax"].get<double>(kSaturatedMaxDefault);
	saturatedMin_ = saturatedMax_ / 2;
	binningGain_ = tuningData["binningGain"].get<double>(1.0);
	flickerPeriod_ = tuningData["flickerPeriod"].get<unsigned int>(kFlickerPeriodDefault) * 1us;
	maxFrameTime_ = tuningData["maxFrameTime"].get<unsigned int>(0) * 1us;
	stretchGain_ = tuningData["stretchGain"].get<double>(0.0);

	if (maxFrameTime_ > 0us) {
		const int64_t limits[] = { kMinFrameTime,
					   static_cast<int64_t>(maxFrameTime_.get<std::micro>()) };
		context.ctrlMap[&controls::FrameDurationLimits] =
			ControlInfo(limits[0], limits[1],
				    ControlValue(Span<const int64_t, 2>(limits)));
	}

	return 0;
}

/* The exposure a frame of at most \a frameTime allows, past the default frame. */
void Agc::setExposureLimit(IPAContext &context, utils::Duration frameTime) const
{
	const auto &cfg = context.configuration.agc;
	const int32_t lines = frameTime / cfg.lineDuration;
	const int32_t extra = std::clamp(lines - cfg.exposureMax, 0,
					 cfg.vblankMax - cfg.vblankMin);

	context.activeState.agc.exposureLimit = cfg.exposureMax + extra;
}

void Agc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	const auto &limits = controls.get(controls::FrameDurationLimits);
	if (!limits || maxFrameTime_ == 0us)
		return;

	const utils::Duration requested = (*limits)[1] * 1us;
	setExposureLimit(context, std::min(maxFrameTime_, requested));
}

/*
 * Split the product of exposure (in lines) and gain into the two: exposure
 * first, gain only for what the exposure cannot give. Longer than the
 * flicker period, the exposure goes down to a multiple of it and the gain
 * makes up the rest, unless the gain would run out: then banding is
 * better than a dark image.
 */
void Agc::split(const IPAContext &context, double total,
		int32_t &exposure, double &gain) const
{
	const auto &agc = context.configuration.agc;

	const int32_t limit = context.activeState.agc.exposureLimit;
	exposure = std::clamp(static_cast<int32_t>(std::lround(total / agc.againMin)),
			      agc.exposureMin, limit);

	/*
	 * Longer frames lower the frame rate: fill the default frame, then
	 * raise the gain up to stretchGain_, and only then stretch the frame.
	 */
	if (exposure > agc.exposureMax && stretchGain_ > agc.againMin) {
		if (total / agc.exposureMax <= stretchGain_)
			exposure = agc.exposureMax;
		else
			exposure = std::clamp(static_cast<int32_t>(std::lround(total / stretchGain_)),
					      agc.exposureMax, limit);
	}

	utils::Duration time = exposure * agc.lineDuration;
	if (flickerPeriod_ > 0us && time >= flickerPeriod_) {
		double periods = std::floor(time / flickerPeriod_);
		int32_t lines = std::max(static_cast<int32_t>(periods * (flickerPeriod_ / agc.lineDuration)),
					 agc.exposureMin);
		if (total / lines <= agc.againMax)
			exposure = lines;
	}

	gain = std::clamp(total / exposure, agc.againMin, agc.againMax);
}

int Agc::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	pending_ = false;
	waited_ = 0;

	setExposureLimit(context, maxFrameTime_);

	auto &agc = context.activeState.agc;
	agc.initial = false;
	if (!configInfo.keepState)
		return 0;

	/*
	 * Back in a mode the sensor already ran in: its last exposure.
	 * Otherwise convert from another one: a binned pixel gathers the light
	 * of several (binningGain per factor of two), keep the exposure time
	 * against motion blur and make up with the gain.
	 */
	const auto &cfg = context.configuration.agc;
	auto &same = agc.byBin[agc.bin - 1];
	utils::Duration time;
	double gain;
	if (same.valid) {
		time = same.time;
		gain = same.gain;
	} else {
		unsigned int from = 0;
		for (unsigned int i = 0; i < 4 && !from; i++)
			if (agc.byBin[i].valid)
				from = i + 1;
		if (!from)
			return 0;
		time = agc.byBin[from - 1].time;
		gain = agc.byBin[from - 1].gain *
		       std::pow(binningGain_, std::log2(static_cast<double>(from) / agc.bin));
	}

	double total = time / cfg.lineDuration * gain;
	int32_t lines;
	split(context, total, lines, gain);
	if (lines * gain < total * 0.99)
		LOG(IPASoftExposure, Debug)
			<< "Mode " << agc.bin << "x underexposed by "
			<< total / (lines * gain);

	agc.initialExposure = lines;
	agc.initialGain = gain;
	agc.initial = true;

	/* The AGC waits for these to show before deciding again. */
	exposureReq_ = lines;
	gainReq_ = gain;
	pending_ = true;
	return 0;
}

void Agc::updateExposure(IPAContext &context, IPAFrameContext &frameContext,
			 double exposureMSV, double saturated)
{
	int32_t &exposure = frameContext.sensor.exposure;
	double &again = frameContext.sensor.gain;

	/* Started with a scaled exposure: that is the request to wait for. */
	auto &state = context.activeState.agc;
	if (state.scaledPending) {
		state.scaledPending = false;
		exposureReq_ = state.startExposure;
		gainReq_ = state.startGain;
		pending_ = true;
		waited_ = 0;
	}

	/* The frame rate asked for may have changed: no frame longer than it allows. */
	if (exposure > state.exposureLimit) {
		int32_t nextExposure;
		double nextGain;

		split(context, exposure * again, nextExposure, nextGain);
		exposure = nextExposure;
		again = nextGain;
		exposureReq_ = nextExposure;
		gainReq_ = nextGain;
		pending_ = true;
		waited_ = 0;
		return;
	}

	/*
	 * A change reaches the sensor output some frames later. Deciding
	 * again on frames still taken with the old values stacks corrections
	 * on top of each other: the exposure overshoots by more than the
	 * hysteresis and oscillates around the target. Keep asking for the
	 * last values until the frame controls show them.
	 */
	satisfied_ = false;
	if (pending_) {
		bool arrived = exposure == exposureReq_ &&
			       std::abs(again - gainReq_) <= 0.03 * gainReq_;
		if (!arrived && ++waited_ < kMaxWait) {
			exposure = exposureReq_;
			again = gainReq_;
			return;
		}
		pending_ = false;
	}

	/*
	 * The MSV is a mean: against the light the shade sets it, and the
	 * sunlit background burns out. Burnt highlights count as too much
	 * light unless the rest would get too dark, and more light is not
	 * asked for while they are already at the limit.
	 */
	bool over = exposureMSV > target_ + kExposureSatisfactory ||
		    (saturated > saturatedMax_ && exposureMSV > msvMin_);
	bool under = exposureMSV < target_ - kExposureSatisfactory &&
		     saturated < saturatedMin_;
	satisfied_ = !over && !under;
	if (satisfied_)
		return;

	/*
	 * Proportional and damped: the MSV is not linear in the light, and a
	 * full correction in one step would overshoot as well. At the ends of
	 * the histogram the MSV says little about how far off it is: take the
	 * largest step.
	 */
	double factor;
	if (over && exposureMSV <= target_ + kExposureSatisfactory)
		factor = 0.85;
	else if (exposureMSV >= kExposureBinsCount - 0.5)
		factor = 0.5;
	else if (exposureMSV <= 1.2)
		factor = 2.0;
	else
		factor = std::clamp(std::sqrt(target_ / exposureMSV), 0.6, 1.6);

	double total = exposure * again * factor;
	int32_t nextExposure;
	double nextGain;
	split(context, total, nextExposure, nextGain);

	LOG(IPASoftExposure, Debug)
		<< "exposureMSV " << exposureMSV << " factor " << factor
		<< " exp " << exposure << " -> " << nextExposure
		<< " again " << again << " -> " << nextGain;

	if (nextExposure == exposure && std::abs(nextGain - again) < 1e-6)
		return;

	exposure = nextExposure;
	again = nextGain;
	exposureReq_ = nextExposure;
	gainReq_ = nextGain;
	pending_ = true;
	waited_ = 0;
}

void Agc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	utils::Duration exposureTime =
		context.configuration.agc.lineDuration * frameContext.sensor.exposure;
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);

	/*
	 * Calculate Mean Sample Value (MSV) according to formula from:
	 * https://www.araa.asn.au/acra/acra2007/papers/paper84final.pdf
	 */
	const auto &histogram = stats->yHistogram;
	const unsigned int blackLevelHistIdx =
		context.activeState.blc.level / (256 / SwIspStats::kYHistogramSize);
	const unsigned int histogramSize =
		SwIspStats::kYHistogramSize - blackLevelHistIdx;
	const unsigned int yHistValsPerBin = histogramSize / kExposureBinsCount;
	const unsigned int yHistValsPerBinMod =
		histogramSize / (histogramSize % kExposureBinsCount + 1);
	int exposureBins[kExposureBinsCount] = {};
	unsigned int denom = 0;
	unsigned int num = 0;

	for (unsigned int i = 0; i < histogramSize; i++) {
		unsigned int idx = (i - (i / yHistValsPerBinMod)) / yHistValsPerBin;
		exposureBins[idx] += histogram[blackLevelHistIdx + i];
	}

	for (unsigned int i = 0; i < kExposureBinsCount; i++) {
		LOG(IPASoftExposure, Debug) << i << ": " << exposureBins[i];
		denom += exposureBins[i];
		num += exposureBins[i] * (i + 1);
	}

	float exposureMSV = (denom == 0 ? 0 : static_cast<float>(num) / denom);

	/* The share of burnt pixels: the top entry of the histogram. */
	const unsigned int total = std::accumulate(histogram.begin(), histogram.end(), 0u);
	double saturated = total ? static_cast<double>(histogram[SwIspStats::kYHistogramSize - 1]) / total : 0;

	updateExposure(context, frameContext, exposureMSV, saturated);

	/*
	 * The light, relative to what the AGC settles on: the mean of the
	 * histogram above the black level (linear, the statistics are taken
	 * before the gamma) over the goal, per unit of exposure time (us)
	 * and gain. Unlike the exposure alone it still measures when the
	 * exposure is at its limit: the flash metering compares frames
	 * without and with the torch on it.
	 */
	double sum = 0;
	unsigned int count = 0;
	for (unsigned int i = blackLevelHistIdx; i < SwIspStats::kYHistogramSize; i++) {
		sum += (i - blackLevelHistIdx + 0.5) * histogram[i];
		count += histogram[i];
	}
	double meanY = count ? sum / count : 0;
	if (satisfied_ && meanY > 0)
		yGoal_ = yGoal_ ? 0.9 * yGoal_ + 0.1 * meanY : meanY;

	double goal = yGoal_ ? yGoal_ : histogramSize * kYGoalDefault;
	double product = exposureTime.get<std::micro>() * frameContext.sensor.gain;
	if (product > 0)
		metadata.set(controls::Lux, static_cast<float>(meanY / goal / product));
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
