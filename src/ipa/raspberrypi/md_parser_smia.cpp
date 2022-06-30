/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi (Trading) Limited
 *
 * md_parser_smia.cpp - SMIA specification based embedded data parser
 */

#include <libcamera/base/log.h>
#include "md_parser.hpp"

using namespace RPiController;
using namespace libcamera;

/*
 * This function goes through the embedded data to find the offsets (not
 * values!), in the data block, where the values of the given registers can
 * subsequently be found.
 *
 * Embedded data tag bytes, from Sony IMX219 datasheet but general to all SMIA
 * sensors, I think.
 */

constexpr unsigned int LineStart = 0x0a;
constexpr unsigned int LineEndTag = 0x07;
constexpr unsigned int RegHiBits = 0xaa;
constexpr unsigned int RegLowBits = 0xa5;
constexpr unsigned int RegValue = 0x5a;
constexpr unsigned int RegSkip = 0x55;

MdParserSmia::MdParserSmia(std::initializer_list<uint32_t> registerList)
{
	for (auto r : registerList)
		offsets_[r] = {};
}

MdParser::Status MdParserSmia::parse(libcamera::Span<const uint8_t> buffer,
				     RegisterMap &registers)
{
	if (reset_) {
		/*
		 * Search again through the metadata for all the registers
		 * requested.
		 */
		ASSERT(bits_per_pixel_);

		for (const auto &kv : offsets_)
			offsets_[kv.first] = {};

		ParseStatus ret = findRegs(buffer);
		/*
		 * > 0 means "worked partially but parse again next time",
		 * < 0 means "hard error".
		 *
		 * In either case, we retry parsing on the next frame.
		 */
		if (ret != ParseOk)
			return ERROR;

		reset_ = false;
	}

	/* Populate the register values requested. */
	registers.clear();
	for (const auto &[reg, offset] : offsets_) {
		if (!offset) {
			reset_ = true;
			return NOTFOUND;
		}
		registers[reg] = buffer[offset.value()];
	}

	return OK;
}

MdParserSmia::ParseStatus MdParserSmia::findRegs(libcamera::Span<const uint8_t> buffer)
{
	ASSERT(offsets_.size());

	if (buffer[0] != LineStart)
		return NoLineStart;

	unsigned int currentOffset = 1; /* after the LINE_START */
	unsigned int currentLineStart = 0, currentLine = 0;
	unsigned int regNum = 0, regsDone = 0;

	while (1) {
		int tag = buffer[currentOffset++];

		if ((bits_per_pixel_ == 10 &&
		     (currentOffset + 1 - currentLineStart) % 5 == 0) ||
		    (bits_per_pixel_ == 12 &&
		     (currentOffset + 1 - currentLineStart) % 3 == 0)) {
			if (buffer[currentOffset++] != RegSkip)
				return BadDummy;
		}

		int dataByte = buffer[currentOffset++];

		if (tag == LineEndTag) {
			if (dataByte != LineEndTag)
				return BadLineEnd;

			if (num_lines_ && ++currentLine == num_lines_)
				return MissingRegs;

			if (line_length_bytes_) {
				currentOffset = currentLineStart + line_length_bytes_;

				/* Require whole line to be in the buffer (if buffer size set). */
				if (buffer.size() &&
				    currentOffset + line_length_bytes_ > buffer.size())
					return MissingRegs;

				if (buffer[currentOffset] != LineStart)
					return NoLineStart;
			} else {
				/* allow a zero line length to mean "hunt for the next line" */
				while (currentOffset < buffer.size() &&
				       buffer[currentOffset] != LineStart)
					currentOffset++;

				if (currentOffset == buffer.size())
					return NoLineStart;
			}

			/* inc current_offset to after LINE_START */
			currentLineStart = currentOffset++;
		} else {
			if (tag == RegHiBits)
				regNum = (regNum & 0xff) | (dataByte << 8);
			else if (tag == RegLowBits)
				regNum = (regNum & 0xff00) | dataByte;
			else if (tag == RegSkip)
				regNum++;
			else if (tag == RegValue) {
				auto reg = offsets_.find(regNum);

				if (reg != offsets_.end()) {
					offsets_[regNum] = currentOffset - 1;

					if (++regsDone == offsets_.size())
						return ParseOk;
				}
				regNum++;
			} else
				return IllegalTag;
		}
	}
}
