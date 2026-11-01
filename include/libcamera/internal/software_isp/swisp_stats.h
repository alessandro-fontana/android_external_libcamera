/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * Statistics data format used by the software ISP and software IPA
 */

#pragma once

#include <array>
#include <stdint.h>

namespace libcamera {

/**
 * \brief Struct that holds the statistics for the Software ISP
 *
 * The struct value types are large enough to not overflow.
 * Should they still overflow for some reason, no check is performed and they
 * wrap around.
 */
struct SwIspStats {
	/**
	 * \brief Holds the sum of all sampled red pixels, in 8-bit units
	 */
	uint64_t sumR_;
	/**
	 * \brief Holds the sum of all sampled green pixels, in 8-bit units
	 */
	uint64_t sumG_;
	/**
	 * \brief Holds the sum of all sampled blue pixels, in 8-bit units
	 */
	uint64_t sumB_;
	/**
	 * \brief Number of bins in the yHistogram
	 */
	static constexpr unsigned int kYHistogramSize = 64;
	/**
	 * \brief Type of the histogram.
	 */
	using Histogram = std::array<uint32_t, kYHistogramSize>;
	/**
	 * \brief A histogram of luminance values
	 */
	Histogram yHistogram;
	/**
	 * \brief Sum of the green differences between neighbouring samples in
	 * the focus window, horizontally and vertically (contrast autofocus)
	 */
	uint64_t sharpness;
	/**
	 * \brief Sum of the green samples in the focus window, to make the
	 * sharpness independent of the exposure
	 */
	uint64_t sharpnessBase;
};

} /* namespace libcamera */
