/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#include "agc.h"

#include <algorithm>
#include <cmath>
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
 * Frames to wait at most for a change to show in the sensor controls
 * reported for a frame, before deciding again anyway.
 */
static constexpr unsigned int kMaxWait = 8;

Agc::Agc()
	: pending_(false), exposureReq_(0), gainReq_(0), waited_(0)
{
}

int Agc::configure([[maybe_unused]] IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	pending_ = false;
	waited_ = 0;
	return 0;
}

void Agc::updateExposure(IPAContext &context, IPAFrameContext &frameContext, double exposureMSV)
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

	if (std::abs(exposureMSV - kExposureOptimal) <= kExposureSatisfactory)
		return;

	/*
	 * Proportional and damped: the MSV is not linear in the light, and a
	 * full correction in one step would overshoot as well. At the ends of
	 * the histogram the MSV says little about how far off it is: take the
	 * largest step.
	 */
	double factor;
	if (exposureMSV >= kExposureBinsCount - 0.5)
		factor = 0.5;
	else if (exposureMSV <= 1.2)
		factor = 2.0;
	else
		factor = std::clamp(std::sqrt(kExposureOptimal / exposureMSV), 0.6, 1.6);

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
	updateExposure(context, frameContext, exposureMSV);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
