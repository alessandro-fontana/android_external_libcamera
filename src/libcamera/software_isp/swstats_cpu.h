/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * CPU based software statistics implementation
 */

#pragma once

#include <array>
#include <stdint.h>
#include <vector>

#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/shared_mem_object.h"
#include "libcamera/internal/software_isp/swisp_stats.h"

namespace libcamera {

class PixelFormat;
struct StreamConfiguration;

class SwStatsCpu
{
public:
	SwStatsCpu();
	~SwStatsCpu() = default;

	bool isValid() const { return sharedStats_.fd().isValid(); }

	const SharedFD &getStatsFD() { return sharedStats_.fd(); }

	const Size &patternSize() { return patternSize_; }

	int configure(const StreamConfiguration &inputCfg);
	void setWindow(const Rectangle &window);
	void setFocusWindow(const std::array<uint16_t, 4> &window);
	void startFrame();
	void finishFrame(uint32_t frame, uint32_t bufferId);

	void processLine0(unsigned int y, const uint8_t *src[])
	{
		if ((y & ySkipMask_) || y < static_cast<unsigned int>(window_.y) ||
		    y >= (window_.y + window_.height))
			return;

		currentY_ = y;
		(this->*stats0_)(src);
	}

	void processLine2(unsigned int y, const uint8_t *src[])
	{
		if ((y & ySkipMask_) || y < static_cast<unsigned int>(window_.y) ||
		    y >= (window_.y + window_.height))
			return;

		(this->*stats2_)(src);
	}

	Signal<uint32_t, uint32_t> statsReady;

private:
	using statsProcessFn = void (SwStatsCpu::*)(const uint8_t *src[]);

	int setupStandardBayerOrder(BayerFormat::Order order);
	/* Bayer 8 bpp unpacked */
	void statsBGGR8Line0(const uint8_t *src[]);
	/* Bayer 10 bpp unpacked */
	void statsBGGR10Line0(const uint8_t *src[]);
	/* Bayer 12 bpp unpacked */
	void statsBGGR12Line0(const uint8_t *src[]);
	/* Bayer 10 bpp packed */
	void statsBGGR10PLine0(const uint8_t *src[]);
	void statsGBRG10PLine0(const uint8_t *src[]);
	void focusLine(unsigned int x, unsigned int g);

	/* Variables set by configure(), used every line */
	statsProcessFn stats0_;
	statsProcessFn stats2_;
	bool swapLines_;

	unsigned int ySkipMask_;

	Rectangle window_;

	/* The focus window, the central ninth of window_ */
	Rectangle focusWindow_;
	unsigned int currentY_;
	unsigned int focusLeft_;
	/* Green samples of the previous sampled line in the focus window */
	std::vector<uint16_t> focusPrev_;
	bool focusPrevValid_;

	Size patternSize_;

	unsigned int xShift_;

	SharedMemObject<SwIspStats> sharedStats_;
	SwIspStats stats_;
};

} /* namespace libcamera */
