/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2021, Google Inc.
 *
 * libcamera Android Camera Request Descriptor
 */

#pragma once

#include <map>
#include <memory>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/mutex.h>
#include <libcamera/base/unique_fd.h>

#include <libcamera/camera.h>
#include <libcamera/framebuffer.h>

#include <hardware/camera3.h>

#include "camera_metadata.h"
#include "hal_framebuffer.h"

class CameraBuffer;
class CameraStream;

class Camera3RequestDescriptor
{
public:
	enum class Status {
		Success,
		Error,
	};

	struct StreamBuffer {
		StreamBuffer(CameraStream *stream,
			     const camera3_stream_buffer_t &buffer,
			     Camera3RequestDescriptor *request);
		~StreamBuffer();

		StreamBuffer(StreamBuffer &&);
		StreamBuffer &operator=(StreamBuffer &&);

		CameraStream *stream;
		buffer_handle_t *camera3Buffer;
		std::unique_ptr<HALFrameBuffer> frameBuffer;
		libcamera::UniqueFD fence;
		Status status = Status::Success;
		libcamera::FrameBuffer *internalBuffer = nullptr;
		const libcamera::FrameBuffer *srcBuffer = nullptr;
		std::unique_ptr<CameraBuffer> dstBuffer;
		Camera3RequestDescriptor *request;

	private:
		LIBCAMERA_DISABLE_COPY(StreamBuffer)
	};

	/* Keeps track of streams requiring post-processing. */
	std::map<CameraStream *, StreamBuffer *> pendingStreamsToProcess_
		LIBCAMERA_TSA_GUARDED_BY(streamsProcessMutex_);
	libcamera::Mutex streamsProcessMutex_;

	Camera3RequestDescriptor(libcamera::Camera *camera,
				 const camera3_capture_request_t *camera3Request);
	~Camera3RequestDescriptor();

	bool isPending() const { return !complete_; }

	uint32_t frameNumber_ = 0;

	std::vector<StreamBuffer> buffers_;

	CameraMetadata settings_;
	std::unique_ptr<libcamera::Request> request_;
	std::unique_ptr<CameraMetadata> resultMetadata_;

	bool complete_ = false;
	Status status_ = Status::Success;

	/* Flash: the modes in force for this request, and what it reports. */
	uint8_t aeMode_ = 1; /* ANDROID_CONTROL_AE_MODE_ON */
	uint8_t flashMode_ = 0; /* ANDROID_FLASH_MODE_OFF */
	bool flashCapture_ = false;
	uint8_t aeState_ = 2; /* ANDROID_CONTROL_AE_STATE_CONVERGED */
	uint8_t flashState_ = 0; /* ANDROID_FLASH_STATE_UNAVAILABLE */

private:
	LIBCAMERA_DISABLE_COPY(Camera3RequestDescriptor)
};
