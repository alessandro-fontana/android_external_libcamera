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
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	void updateExposure(IPAContext &context, IPAFrameContext &frameContext,
			    double exposureMSV, double saturated);
	void split(const IPAContext &context, double total,
		   int32_t &exposure, double &gain) const;
	void setExposureLimit(IPAContext &context, utils::Duration frameTime) const;

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
	/* The AGC was satisfied with the last frame */
	bool satisfied_ = false;
	/* Mean of the linear histogram the AGC settles on, in bins */
	double yGoal_ = 0;
	/* Mains flicker period to keep the exposure a multiple of, 0 = off */
	utils::Duration flickerPeriod_;
	/* Longest frame in dim light, from the tuning file, 0 = the sensor's default */
	utils::Duration maxFrameTime_;
	double stretchGain_;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
