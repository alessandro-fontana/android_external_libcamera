/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * JPEG Post Processor
 */

#pragma once

#include "../post_processor.h"
#include "encoder_libjpeg.h"
#include "thumbnailer.h"

#include <vector>

#include <libcamera/geometry.h>

class CameraDevice;

class PostProcessorJpeg : public PostProcessor
{
public:
	PostProcessorJpeg(CameraDevice *const device);

	int configure(const libcamera::StreamConfiguration &incfg,
		      const libcamera::StreamConfiguration &outcfg) override;
	void process(Camera3RequestDescriptor::StreamBuffer *streamBuffer) override;

private:
	int encodeScaled(Camera3RequestDescriptor::StreamBuffer *streamBuffer,
			 libcamera::Span<const uint8_t> exifData,
			 unsigned int quality);
	void generateThumbnail(const libcamera::FrameBuffer &source,
			       const libcamera::Size &targetSize,
			       unsigned int quality,
			       std::vector<unsigned char> *thumbnail);

	CameraDevice *const cameraDevice_;
	std::unique_ptr<Encoder> encoder_;
	libcamera::Size streamSize_;
	/* An XRGB source of another size, cropped and scaled before encoding */
	bool scale_ = false;
	libcamera::Size sourceSize_;
	unsigned int sourceStride_ = 0;
	std::vector<uint8_t> scaled_;
	EncoderLibJpeg thumbnailEncoder_;
	Thumbnailer thumbnailer_;
};
