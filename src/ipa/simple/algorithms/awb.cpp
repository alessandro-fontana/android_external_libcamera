/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Auto white balance
 */

#include "awb.h"

#include <algorithm>
#include <numeric>
#include <stdint.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libipa/colours.h"
#include "simple/ipa_context.h"

#include "control_ids.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftAwb)

namespace ipa::soft::algorithms {

int Awb::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	for (const YamlObject &point : tuningData["whiteCurve"].asList()) {
		const auto ct = point["ct"].get<float>();
		const auto r = point["r"].get<float>();
		const auto b = point["b"].get<float>();
		if (!ct || !r || !b) {
			LOG(IPASoftAwb, Error) << "Invalid point in whiteCurve";
			return -EINVAL;
		}
		whiteCurve_.push_back({ *ct, *r, *b });
	}

	if (!whiteCurve_.empty() && whiteCurve_.size() != 2) {
		LOG(IPASoftAwb, Error) << "whiteCurve takes two points";
		return -EINVAL;
	}

	return 0;
}

int Awb::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	/* A still capture in another sensor mode keeps the preview's gains. */
	if (configInfo.keepState)
		return 0;

	auto &gains = context.activeState.awb.gains;
	gains = { { 1.0, 1.0, 1.0 } };

	return 0;
}

void Awb::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] DebayerParams *params)
{
	auto &gains = context.activeState.awb.gains;
	/* Just report, the gains are applied in LUT algorithm. */
	frameContext.gains.red = gains.r();
	frameContext.gains.blue = gains.b();
}

void Awb::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	const SwIspStats::Histogram &histogram = stats->yHistogram;
	const uint8_t blackLevel = context.activeState.blc.level;

	const float mdGains[] = {
		static_cast<float>(frameContext.gains.red),
		static_cast<float>(frameContext.gains.blue)
	};
	metadata.set(controls::ColourGains, mdGains);

	/*
	 * Black level must be subtracted to get the correct AWB ratios, they
	 * would be off if they were computed from the whole brightness range
	 * rather than from the sensor range.
	 */
	const uint64_t nPixels = std::accumulate(
		histogram.begin(), histogram.end(), 0);
	/*
	 * Every statistics sample adds one red, one (averaged) green and one
	 * blue value, and one histogram entry: each sum holds nPixels values
	 * and carries the whole black level offset.
	 */
	const uint64_t offset = blackLevel * nPixels;
	const uint64_t sumR = stats->sumR_ > offset ? stats->sumR_ - offset : 0;
	const uint64_t sumG = stats->sumG_ > offset ? stats->sumG_ - offset : 0;
	const uint64_t sumB = stats->sumB_ > offset ? stats->sumB_ - offset : 0;

	/*
	 * Calculate red and blue gains for AWB.
	 * Clamp max gain at 4.0, this also avoids 0 division.
	 */
	auto &gains = context.activeState.awb.gains;
	gains = { {
		sumR <= sumG / 4 ? 4.0f : static_cast<float>(sumG) / sumR,
		1.0,
		sumB <= sumG / 4 ? 4.0f : static_cast<float>(sumG) / sumB,
	} };

	RGB<double> rgbGains{ { 1 / gains.r(), 1 / gains.g(), 1 / gains.b() } };
	context.activeState.awb.temperatureK = estimateCCT(rgbGains);

	/*
	 * The grey world follows what fills the scene: a bright coloured area
	 * passes for the light. Lights lie on the white line of the sensor, so
	 * take the point of the line nearest to the estimate.
	 */
	if (!whiteCurve_.empty() && sumG && sumR && sumB) {
		const White &w0 = whiteCurve_[0];
		const White &w1 = whiteCurve_[1];
		const float dr = w1.r - w0.r;
		const float db = w1.b - w0.b;
		const float r = static_cast<float>(sumR) / sumG;
		const float b = static_cast<float>(sumB) / sumG;
		const float t = std::clamp(((r - w0.r) * dr + (b - w0.b) * db) /
					   (dr * dr + db * db), 0.0f, 1.0f);

		gains = { { 1 / (w0.r + t * dr), 1.0, 1 / (w0.b + t * db) } };
		/* Temperature is linear on the line in mireds. */
		context.activeState.awb.temperatureK =
			1 / (1 / w0.ct + t * (1 / w1.ct - 1 / w0.ct));
	}
	metadata.set(controls::ColourTemperature, context.activeState.awb.temperatureK);

	LOG(IPASoftAwb, Debug)
		<< "gain R/B: " << gains << "; temperature: "
		<< context.activeState.awb.temperatureK;
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
