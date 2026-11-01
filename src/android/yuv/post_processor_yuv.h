/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Post Processor using libyuv
 */

#pragma once

#include <vector>

#include "../post_processor.h"

#include <libcamera/geometry.h>

class PostProcessorYuv : public PostProcessor
{
public:
	/*
	 * fullRange selects full range BT.601 (JFIF) for the YUV output of
	 * RGB sources, limited range BT.601 otherwise.
	 */
	explicit PostProcessorYuv(bool fullRange)
		: fullRange_(fullRange)
	{
	}

	int configure(const libcamera::StreamConfiguration &incfg,
		      const libcamera::StreamConfiguration &outcfg) override;
	void process(Camera3RequestDescriptor::StreamBuffer *streamBuffer) override;

private:
	bool isValidBuffers(const libcamera::FrameBuffer &source,
			    const CameraBuffer &destination) const;
	void calculateLengths(const libcamera::StreamConfiguration &inCfg,
			      const libcamera::StreamConfiguration &outCfg);
	int argbToNV12(const uint8_t *src, int srcStride, uint8_t *dstY,
		       int dstStrideY, uint8_t *dstUV, int dstStrideUV,
		       int width, int height);

	bool fullRange_;

	libcamera::Size sourceSize_;
	libcamera::Size destinationSize_;
	libcamera::PixelFormat sourceFormat_;
	libcamera::PixelFormat destinationFormat_;
	std::vector<uint8_t> scaled_;
	std::vector<uint8_t> chroma_;
	unsigned int sourceLength_[3] = {};
	unsigned int sourceStride_[3] = {};
	unsigned int sourceNumPlanes_;
	unsigned int destinationNumPlanes_;
};
