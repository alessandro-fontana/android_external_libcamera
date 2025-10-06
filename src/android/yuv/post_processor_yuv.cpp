/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Post Processor using libyuv
 */

#include "post_processor_yuv.h"

#include <utility>

#include <libyuv/convert.h>
#include <libyuv/scale.h>

#include <libcamera/base/log.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(YUV)

namespace {

/**
 * \var supportedConversions
 * \brief list of supported output pixel formats for an input pixel format
 */
const std::map<PixelFormat, const std::vector<PixelFormat>> supportedConversions = {
	{ formats::YUYV, { formats::NV12 } },
};

} /* namespace */

int PostProcessorYuv::configure(const StreamConfiguration &inCfg,
				const StreamConfiguration &outCfg)
{
	if (inCfg.pixelFormat != outCfg.pixelFormat) {
		const auto it = supportedConversions.find(inCfg.pixelFormat);
		if (it == supportedConversions.end()) {
			LOG(YUV, Error) << "Unsupported source format " << inCfg.pixelFormat;
			return -EINVAL;
		}

		std::vector<PixelFormat> outFormats = it->second;
		const auto &match = std::find(outFormats.begin(), outFormats.end(), outCfg.pixelFormat);
		if (match == outFormats.end()) {
			LOG(YUV, Error) << "Requested pixel format conversion is not supported"
					<< " (from " << inCfg.pixelFormat
					<< " to " << outCfg.pixelFormat << ")";
			return -EINVAL;
		}
	} else {
		if (inCfg.pixelFormat != formats::NV12) {
			LOG(YUV, Error) << "Unsupported format " << inCfg.pixelFormat
					<< " (only NV12 is supported for scaling)";
			return -EINVAL;
		}
	}

	if (inCfg.size < outCfg.size) {
		LOG(YUV, Error) << "Up-scaling is not supported"
				<< " (from " << inCfg.size
				<< " to " << outCfg.size << ")";
		return -EINVAL;
	}

	calculateLengths(inCfg, outCfg);
	return 0;
}

void PostProcessorYuv::process(Camera3RequestDescriptor::StreamBuffer *streamBuffer)
{
	const FrameBuffer &source = *streamBuffer->srcBuffer;
	CameraBuffer *destination = streamBuffer->dstBuffer.get();

	if (!isValidBuffers(source, *destination)) {
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	const MappedFrameBuffer sourceMapped(&source, MappedFrameBuffer::MapFlag::Read);
	if (!sourceMapped.isValid()) {
		LOG(YUV, Error) << "Failed to mmap camera frame buffer";
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	int ret = 0;
	switch (sourceFormat_) {
	case formats::NV12:
		ret = libyuv::NV12Scale(sourceMapped.planes()[0].data(),
					sourceStride_[0],
					sourceMapped.planes()[1].data(),
					sourceStride_[1],
					sourceSize_.width, sourceSize_.height,
					destination->plane(0).data(),
					destinationStride_[0],
					destination->plane(1).data(),
					destinationStride_[1],
					destinationSize_.width,
					destinationSize_.height,
					libyuv::FilterMode::kFilterBilinear);
		break;
	case formats::YUYV:
		ret = libyuv::YUY2ToNV12(sourceMapped.planes()[0].data(),
					 sourceStride_[0],
					 destination->plane(0).data(),
					 destinationStride_[0],
					 destination->plane(1).data(),
					 destinationStride_[1],
					 destinationSize_.width,
					 destinationSize_.height);
		break;
	default:
		LOG(YUV, Error) << "Unsupported source format " << sourceFormat_;
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		break;
	}

	if (ret) {
		LOG(YUV, Error) << "Libyuv operation failure: " << ret;
		processComplete.emit(streamBuffer, PostProcessor::Status::Error);
		return;
	}

	processComplete.emit(streamBuffer, PostProcessor::Status::Success);
}

bool PostProcessorYuv::isValidBuffers(const FrameBuffer &source,
				      const CameraBuffer &destination) const
{
	if (source.planes().size() != sourceNumPlanes_) {
		LOG(YUV, Error) << "Invalid number of source planes: "
				<< source.planes().size();
		return false;
	}
	if (destination.numPlanes() != destinationNumPlanes_) {
		LOG(YUV, Error) << "Invalid number of destination planes: "
				<< destination.numPlanes();
		return false;
	}

	for (unsigned int i = 0; i < sourceNumPlanes_; i++) {
		if (source.planes()[i].length < sourceLength_[i]) {
			LOG(YUV, Error)
				<< "The source planes lengths are too small, "
				<< "actual size[" << i << "]="
				<< source.planes()[i].length
				<< ", expected size[" << i << "]="
				<< sourceLength_[i];
			return false;
		}
	}
	for (unsigned int i = 0; i < destinationNumPlanes_; i++) {
		if (destination.plane(i).size() < destinationLength_[i]) {
			LOG(YUV, Error)
				<< "The destination planes lengths are too small, "
				<< "actual size[" << i << "]="
				<< destination.plane(i).size()
				<< ", expected size[" << i << "]="
				<< sourceLength_[i];
			return false;
		}
	}

	return true;
}

void PostProcessorYuv::calculateLengths(const StreamConfiguration &inCfg,
					const StreamConfiguration &outCfg)
{
	sourceSize_ = inCfg.size;
	destinationSize_ = outCfg.size;
	sourceFormat_ = inCfg.pixelFormat;
	destinationFormat_ = outCfg.pixelFormat;

	// const PixelFormatInfo &nv12Info = PixelFormatInfo::info(formats::NV12);
	const PixelFormatInfo &sourceInfo = PixelFormatInfo::info(sourceFormat_);
	sourceNumPlanes_ = sourceInfo.numPlanes();
	for (unsigned int i = 0; i < sourceInfo.numPlanes(); i++) {
		sourceStride_[i] = inCfg.stride;
		sourceLength_[i] = sourceInfo.planeSize(sourceSize_.height, i,
							sourceStride_[i]);
	}

	const PixelFormatInfo &destinationInfo = PixelFormatInfo::info(destinationFormat_);
	destinationNumPlanes_ = destinationInfo.numPlanes();
	for (unsigned int i = 0; i < destinationInfo.numPlanes(); i++) {
		destinationStride_[i] = destinationInfo.stride(destinationSize_.width, i, 1);
		destinationLength_[i] = destinationInfo.planeSize(destinationSize_.height, i,
								  destinationStride_[i]);
	}
}
