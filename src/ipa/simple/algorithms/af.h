/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Contrast autofocus
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Af : public Algorithm
{
public:
	Af() = default;
	~Af() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	enum class State {
		Waiting,
		Coarse,
		Fine,
		Climb,
		Focused,
	};

	void moveTo(IPAContext &context, int32_t position);
	void startSweep(IPAContext &context, State state, int32_t from,
			int32_t to, int32_t step);
	void sweep(IPAContext &context, double sharpness);
	void startClimb(IPAContext &context, double sharpness);
	void climb(IPAContext &context, double sharpness);
	void finish(IPAContext &context);
	void watch(IPAContext &context, double sharpness, double brightness);

	State state_;
	unsigned int settle_;

	int32_t position_;
	int32_t step_;
	int32_t end_;
	int32_t bestPosition_;
	double best_;
	double worst_;
	unsigned int falling_;
	int32_t climbStart_;
	int32_t climbDirection_;

	double reference_;
	double last_;
	double referenceBrightness_;
	double lastBrightness_;
	unsigned int changed_;
	unsigned int steady_;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
