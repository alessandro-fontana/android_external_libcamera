/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Post Processor using libyuv
 */

#pragma once

#include "../post_processor.h"

#include <libcamera/geometry.h>

class PostProcessorYuv : public PostProcessor
{
public:
	PostProcessorYuv() = default;

	int configure(const libcamera::StreamConfiguration &incfg,
		      const libcamera::StreamConfiguration &outcfg) override;
	void process(Camera3RequestDescriptor::StreamBuffer *streamBuffer) override;

private:
	bool isValidBuffers(const libcamera::FrameBuffer &source,
			    const CameraBuffer &destination) const;
	void calculateLengths(const libcamera::StreamConfiguration &inCfg,
			      const libcamera::StreamConfiguration &outCfg);

	libcamera::Size sourceSize_;
	libcamera::Size destinationSize_;
	libcamera::PixelFormat sourceFormat_;
	libcamera::PixelFormat destinationFormat_;
	unsigned int sourceLength_[3] = {};
	unsigned int destinationLength_[3] = {};
	unsigned int sourceStride_[3] = {};
	unsigned int destinationStride_[3] = {};
	unsigned int sourceNumPlanes_;
	unsigned int destinationNumPlanes_;
};
