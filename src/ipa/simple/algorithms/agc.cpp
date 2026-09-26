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
	return 0;
}

int Agc::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	pending_ = false;
	waited_ = 0;

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

	if (gain > cfg.againMax) {
		time = time * (gain / cfg.againMax);
		gain = cfg.againMax;
	} else if (gain < cfg.againMin) {
		time = time * (gain / cfg.againMin);
		gain = cfg.againMin;
	}

	int32_t lines = std::clamp(static_cast<int32_t>(std::lround(time / cfg.lineDuration)),
				   cfg.exposureMin, cfg.exposureMax);
	if (lines == cfg.exposureMax && time > lines * cfg.lineDuration)
		LOG(IPASoftExposure, Debug)
			<< "Mode " << agc.bin << "x underexposed by "
			<< time / (lines * cfg.lineDuration);

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
	const auto &agc = context.configuration.agc;
	int32_t &exposure = frameContext.sensor.exposure;
	double &again = frameContext.sensor.gain;

	/*
	 * A change reaches the sensor output some frames later. Deciding
	 * again on frames still taken with the old values stacks corrections
	 * on top of each other: the exposure overshoots by more than the
	 * hysteresis and oscillates around the target. Keep asking for the
	 * last values until the frame controls show them.
	 */
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
	if (!over && !under)
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

	/* Exposure first, gain only for what the exposure cannot give. */
	double total = exposure * again * factor;
	int32_t nextExposure = std::clamp(static_cast<int32_t>(std::lround(total / agc.againMin)),
					  agc.exposureMin, agc.exposureMax);
	double nextGain = std::clamp(total / nextExposure, agc.againMin, agc.againMax);

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
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
