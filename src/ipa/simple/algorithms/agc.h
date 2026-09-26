/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	void updateExposure(IPAContext &context, IPAFrameContext &frameContext,
			    double exposureMSV, double saturated);

	/* The last exposure and gain asked for, and frames spent waiting for them. */
	bool pending_;
	int32_t exposureReq_;
	double gainReq_;
	unsigned int waited_;

	/* From the tuning file: MSV target and highlight protection. */
	float target_;
	float msvMin_;
	double saturatedMax_;
	double saturatedMin_;
	/* How much brighter a pixel binned by two is, from the tuning file */
	double binningGain_ = 1.0;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
