/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * bayer_format.cpp - Class to represent Bayer formats
 */

#include "libcamera/internal/bayer_format.h"

#include <algorithm>
#include <map>
#include <unordered_map>

#include <linux/media-bus-format.h>

#include <libcamera/pixel_format.h>
#include <libcamera/transform.h>

/**
 * \file bayer_format.h
 * \brief Class to represent Bayer formats and manipulate them
 */

namespace libcamera {

/**
 * \class BayerFormat
 * \brief Class to represent a raw image Bayer format
 *
 * This class encodes the different Bayer formats in such a way that they can
 * be easily manipulated. For example, the bit depth or Bayer order can be
 * easily altered - the Bayer order can even be "transformed" in the same
 * manner as happens in many sensors when their horizontal or vertical "flip"
 * controls are set.
 */

/**
 * \enum BayerFormat::Order
 * \brief The order of the colour channels in the Bayer pattern
 *
 * \var BayerFormat::BGGR
 * \brief B then G on the first row, G then R on the second row.
 * \var BayerFormat::GBRG
 * \brief G then B on the first row, R then G on the second row.
 * \var BayerFormat::GRBG
 * \brief G then R on the first row, B then G on the second row.
 * \var BayerFormat::RGGB
 * \brief R then G on the first row, G then B on the second row.
 * \var BayerFormat::MONO
 * \brief Monochrome image data, there is no colour filter array.
 */

/**
 * \enum BayerFormat::Packing
 * \brief Different types of packing that can be applied to a BayerFormat
 *
 * \var BayerFormat::None
 * \brief No packing
 * \var BayerFormat::CSI2Packed
 * \brief Format uses MIPI CSI-2 style packing
 * \var BayerFormat::IPU3Packed
 * \brief Format uses IPU3 style packing
 */

namespace {

/* Define a slightly arbitrary ordering so that we can use a std::map. */
struct BayerFormatComparator {
	constexpr bool operator()(const BayerFormat &lhs, const BayerFormat &rhs) const
	{
		if (lhs.bitDepth < rhs.bitDepth)
			return true;
		else if (lhs.bitDepth > rhs.bitDepth)
			return false;

		if (lhs.order < rhs.order)
			return true;
		else if (lhs.order > rhs.order)
			return false;

		if (lhs.packing < rhs.packing)
			return true;
		else
			return false;
	}
};

const std::map<BayerFormat, V4L2PixelFormat, BayerFormatComparator> bayerToV4l2{
	{ { BayerFormat::BGGR, 8, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR8) },
	{ { BayerFormat::GBRG, 8, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG8) },
	{ { BayerFormat::GRBG, 8, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG8) },
	{ { BayerFormat::RGGB, 8, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB8) },
	{ { BayerFormat::BGGR, 10, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10) },
	{ { BayerFormat::GBRG, 10, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10) },
	{ { BayerFormat::GRBG, 10, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10) },
	{ { BayerFormat::RGGB, 10, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10) },
	{ { BayerFormat::BGGR, 10, BayerFormat::CSI2Packed }, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR10P) },
	{ { BayerFormat::GBRG, 10, BayerFormat::CSI2Packed }, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG10P) },
	{ { BayerFormat::GRBG, 10, BayerFormat::CSI2Packed }, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10P) },
	{ { BayerFormat::RGGB, 10, BayerFormat::CSI2Packed }, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB10P) },
	{ { BayerFormat::BGGR, 10, BayerFormat::IPU3Packed }, V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SBGGR10) },
	{ { BayerFormat::GBRG, 10, BayerFormat::IPU3Packed }, V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SGBRG10) },
	{ { BayerFormat::GRBG, 10, BayerFormat::IPU3Packed }, V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SGRBG10) },
	{ { BayerFormat::RGGB, 10, BayerFormat::IPU3Packed }, V4L2PixelFormat(V4L2_PIX_FMT_IPU3_SRGGB10) },
	{ { BayerFormat::BGGR, 12, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12) },
	{ { BayerFormat::GBRG, 12, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12) },
	{ { BayerFormat::GRBG, 12, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12) },
	{ { BayerFormat::RGGB, 12, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12) },
	{ { BayerFormat::BGGR, 12, BayerFormat::CSI2Packed }, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR12P) },
	{ { BayerFormat::GBRG, 12, BayerFormat::CSI2Packed }, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG12P) },
	{ { BayerFormat::GRBG, 12, BayerFormat::CSI2Packed }, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG12P) },
	{ { BayerFormat::RGGB, 12, BayerFormat::CSI2Packed }, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB12P) },
	{ { BayerFormat::BGGR, 16, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SBGGR16) },
	{ { BayerFormat::GBRG, 16, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SGBRG16) },
	{ { BayerFormat::GRBG, 16, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SGRBG16) },
	{ { BayerFormat::RGGB, 16, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_SRGGB16) },
	{ { BayerFormat::MONO, 8, BayerFormat::None }, V4L2PixelFormat(V4L2_PIX_FMT_GREY) },
	{ { BayerFormat::MONO, 10, BayerFormat::CSI2Packed }, V4L2PixelFormat(V4L2_PIX_FMT_Y10P) },
};

const std::unordered_map<unsigned int, BayerFormat> mbusCodeToBayer{
	{ MEDIA_BUS_FMT_SBGGR8_1X8, { BayerFormat::BGGR, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, { BayerFormat::GBRG, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, { BayerFormat::GRBG, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, { BayerFormat::RGGB, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SBGGR10_ALAW8_1X8, { BayerFormat::BGGR, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGBRG10_ALAW8_1X8, { BayerFormat::GBRG, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGRBG10_ALAW8_1X8, { BayerFormat::GRBG, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SRGGB10_ALAW8_1X8, { BayerFormat::RGGB, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8, { BayerFormat::BGGR, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8, { BayerFormat::GBRG, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8, { BayerFormat::GRBG, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8, { BayerFormat::RGGB, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_BE, { BayerFormat::BGGR, 10, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE, { BayerFormat::BGGR, 10, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_BE, { BayerFormat::BGGR, 10, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_LE, { BayerFormat::BGGR, 10, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, { BayerFormat::BGGR, 10, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, { BayerFormat::GBRG, 10, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, { BayerFormat::GRBG, 10, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, { BayerFormat::RGGB, 10, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, { BayerFormat::BGGR, 12, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, { BayerFormat::GBRG, 12, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, { BayerFormat::GRBG, 12, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, { BayerFormat::RGGB, 12, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SBGGR14_1X14, { BayerFormat::BGGR, 14, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGBRG14_1X14, { BayerFormat::GBRG, 14, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGRBG14_1X14, { BayerFormat::GRBG, 14, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SRGGB14_1X14, { BayerFormat::RGGB, 14, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SBGGR16_1X16, { BayerFormat::BGGR, 16, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGBRG16_1X16, { BayerFormat::GBRG, 16, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SGRBG16_1X16, { BayerFormat::GRBG, 16, BayerFormat::None } },
	{ MEDIA_BUS_FMT_SRGGB16_1X16, { BayerFormat::RGGB, 16, BayerFormat::None } },
	{ MEDIA_BUS_FMT_Y8_1X8, { BayerFormat::MONO, 8, BayerFormat::None } },
	{ MEDIA_BUS_FMT_Y10_1X10, { BayerFormat::MONO, 10, BayerFormat::None } },
};

} /* namespace */

/**
 * \fn BayerFormat::BayerFormat()
 * \brief Construct an empty (and invalid) BayerFormat
 */

/**
 * \fn BayerFormat::BayerFormat(Order o, uint8_t b, Packing p)
 * \brief Construct a BayerFormat from explicit values
 * \param[in] o The order of the Bayer pattern
 * \param[in] b The bit depth of the Bayer samples
 * \param[in] p The type of packing applied to the pixel values
 */

/**
 * \brief Retrieve the media bus code associated with a BayerFormat
 *
 * The media bus code numeric identifiers are defined by the V4L2 specification.
 */
unsigned int BayerFormat::toMbusCode() const
{
	const auto it = std::find_if(mbusCodeToBayer.begin(), mbusCodeToBayer.end(),
				     [this](const auto &p) {
					     return p.second == *this;
				     });

	if (it == mbusCodeToBayer.end())
		return it->first;

	return 0;
}

/**
 * \brief Retrieve the BayerFormat associated with a media bus code
 * \param[in] mbusCode The media bus code to convert into a BayerFormat
 *
 * The media bus code numeric identifiers are defined by the V4L2 specification.
 */
const BayerFormat &BayerFormat::fromMbusCode(unsigned int mbusCode)
{
	static BayerFormat empty;

	const auto it = mbusCodeToBayer.find(mbusCode);
	if (it == mbusCodeToBayer.end())
		return empty;
	else
		return it->second;
}

/**
 * \fn BayerFormat::isValid()
 * \brief Return whether a BayerFormat is valid
 */

/**
 * \brief Assemble and return a readable string representation of the
 * BayerFormat
 * \return A string describing the BayerFormat
 */
std::string BayerFormat::toString() const
{
	std::string result;

	static const char *orderStrings[] = {
		"BGGR",
		"GBRG",
		"GRBG",
		"RGGB",
		"MONO"
	};
	if (isValid() && order <= MONO)
		result = orderStrings[order];
	else
		return "INVALID";

	result += "-" + std::to_string(bitDepth);

	if (packing == CSI2Packed)
		result += "-CSI2P";
	else if (packing == IPU3Packed)
		result += "-IPU3P";

	return result;
}

/**
 * \brief Compare two BayerFormats for equality
 * \return True if order, bitDepth and packing are equal, or false otherwise
 */
bool operator==(const BayerFormat &lhs, const BayerFormat &rhs)
{
	return lhs.order == rhs.order && lhs.bitDepth == rhs.bitDepth &&
	       lhs.packing == rhs.packing;
}

/**
 * \fn bool operator!=(const BayerFormat &lhs, const BayerFormat &rhs)
 * \brief Compare two BayerFormats for inequality
 * \return True if either order, bitdepth or packing are not equal, or false
 * otherwise
 */

/**
 * \brief Convert a BayerFormat into the corresponding V4L2PixelFormat
 * \return The V4L2PixelFormat corresponding to this BayerFormat
 */
V4L2PixelFormat BayerFormat::toV4L2PixelFormat() const
{
	const auto it = bayerToV4l2.find(*this);
	if (it != bayerToV4l2.end())
		return it->second;

	return V4L2PixelFormat();
}

/**
 * \brief Convert \a v4l2Format to the corresponding BayerFormat
 * \param[in] v4l2Format The raw format to convert into a BayerFormat
 * \return The BayerFormat corresponding to \a v4l2Format
 */
BayerFormat BayerFormat::fromV4L2PixelFormat(V4L2PixelFormat v4l2Format)
{
	auto it = std::find_if(bayerToV4l2.begin(), bayerToV4l2.end(),
			       [v4l2Format](const auto &i) {
				       return i.second == v4l2Format;
			       });
	if (it != bayerToV4l2.end())
		return it->first;

	return BayerFormat();
}

/**
 * \brief Convert a BayerFormat into the corresponding PixelFormat
 * \return The PixelFormat corresponding to this BayerFormat
 */
PixelFormat BayerFormat::toPixelFormat() const
{
	const auto it = bayerToV4l2.find(*this);
	if (it != bayerToV4l2.end())
		return PixelFormat(it->second);

	return PixelFormat();
}

/**
 * \brief Apply a transform to this BayerFormat
 * \param[in] t The transform to apply
 *
 * Appplying a transform to an image stored in a Bayer format affects the Bayer
 * order. For example, performing a horizontal flip on the Bayer pattern
 * RGGB causes the RG rows of pixels to become GR, and the GB rows to become BG.
 * The transformed image would have a GRBG order. The bit depth and modifiers
 * are not affected.
 *
 * Horizontal and vertical flips are applied before transpose.
 *
 * \return The transformed Bayer format
 */
BayerFormat BayerFormat::transform(Transform t) const
{
	BayerFormat result = *this;

	if (order == MONO)
		return result;

	/*
	 * Observe that flipping bit 0 of the Order enum performs a horizontal
	 * mirror on the Bayer pattern (e.g. RGGB goes to GRBG). Similarly,
	 * flipping bit 1 performs a vertical mirror operation on it. Hence:
	 */
	if (!!(t & Transform::HFlip))
		result.order = static_cast<Order>(result.order ^ 1);
	if (!!(t & Transform::VFlip))
		result.order = static_cast<Order>(result.order ^ 2);

	if (!!(t & Transform::Transpose) && result.order == 1)
		result.order = static_cast<Order>(2);
	else if (!!(t & Transform::Transpose) && result.order == 2)
		result.order = static_cast<Order>(1);

	return result;
}

/**
 * \var BayerFormat::order
 * \brief The order of the colour channels in the Bayer pattern
 */

/**
 * \var BayerFormat::bitDepth
 * \brief The bit depth of the samples in the Bayer pattern
 */

/**
 * \var BayerFormat::packing
 * \brief Any packing scheme applied to this BayerFormat
 */

} /* namespace libcamera */
