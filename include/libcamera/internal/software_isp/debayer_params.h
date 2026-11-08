/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023-2025 Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * DebayerParams header
 */

#pragma once

#include <array>
#include <stdint.h>

namespace libcamera {

struct DebayerParams {
	static constexpr unsigned int kRGBLookupSize = 256;

	struct CcmColumn {
		int16_t r;
		int16_t g;
		int16_t b;
	};

	using LookupTable = std::array<uint8_t, kRGBLookupSize>;
	using CcmLookupTable = std::array<CcmColumn, kRGBLookupSize>;

	/*
	 * Color lookup tables when CCM is not used.
	 *
	 * Each color of a debayered pixel is amended by the corresponding
	 * value in the given table.
	 */
	LookupTable red;
	LookupTable green;
	LookupTable blue;

	/*
	 * Color and gamma lookup tables when CCM is used.
	 *
	 * Each of the CcmLookupTable's corresponds to a CCM column; together they
	 * make a complete 3x3 CCM lookup table. The CCM is applied on debayered
	 * pixels and then the gamma lookup table is used to set the resulting
	 * values of all the three colors.
	 */
	CcmLookupTable redCcm;
	CcmLookupTable greenCcm;
	CcmLookupTable blueCcm;
	LookupTable gammaLut;

	/*
	 * Autofocus window: x, y, width, height as fractions of the processed
	 * window, in 1/65535 units. A zero width keeps the default window.
	 */
	std::array<uint16_t, 4> focusWindow;

	/*
	 * The white line of the sensor for the grey statistics: R/G and B/G
	 * at one end, their change to the other end, and the distance from
	 * the line a sample may have, all in 1/1024. A zero distance turns
	 * the grey statistics off.
	 */
	std::array<int32_t, 5> greyLine;
	/* Black level, in 8-bit units, for the grey statistics */
	uint8_t greyBlack;
};

} /* namespace libcamera */
