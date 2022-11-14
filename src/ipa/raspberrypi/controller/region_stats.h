/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * region_stats.h - Raspberry Pi region based statistics container
 */
#pragma once

#include <array>
#include <stdint.h>
#include <vector>

namespace RPiController {

template<typename T>
class RegionStats
{
public:
	RegionStats()
		: numRows_(0), numCols_(0), numFloating_(0)
	{
	}

	void init(unsigned int numRows, unsigned int numCols = 1, unsigned int numFloating = 0)
	{
		numRows_ = numRows;
		numCols_ = numCols;
		numFloating_ = numFloating;
		regions_.resize(numRows_ * numCols_ + numFloating_);
	}

	unsigned int numRegions() const
	{
		return numRows_ * numCols_;
	}

	unsigned int numFloatingRegions() const
	{
		return numFloating_;
	}

	void set(unsigned int index, const T &value, uint32_t counted, uint32_t uncounted)
	{
		if (index >= numRegions())
			return;
		set_(index, value, counted, uncounted);
	}

	void set(unsigned int row, unsigned int col, const T &value, uint32_t counted, uint32_t uncounted)
	{
		set(row * numCols_ + col, value, counted, uncounted);
	}

	void setFloating(unsigned int index, const T &value, uint32_t counted, uint32_t uncounted)
	{
		if (index >= numFloatingRegions())
			return;
		set(numRegions() + index, value, counted, uncounted);
	}

	const T &get(unsigned int index, uint32_t &counted, uint32_t &uncounted) const
	{
		counted = uncounted = 0;
		if (index >= numRegions())
			return default_;
		return get_(index, counted, uncounted);
	}

	const T &get(unsigned int row, unsigned int col, uint32_t &counted, uint32_t &uncounted) const
	{
		return get(row * numCols_ + col, counted, uncounted);
	}

	const T &getFloating(unsigned int index, uint32_t &counted, uint32_t &uncounted) const
	{
		counted = uncounted = 0;
		if (index >= numFloatingRegions())
			return default_;
		return get_(numRegions() + index, counted, uncounted);
	}

private:
	void set_(unsigned int index, const T &value, uint32_t counted, uint32_t uncounted)
	{
		regions_[index] = { value, counted, uncounted };
	}

	const T &get_(unsigned int index, uint32_t &counted, uint32_t &uncounted) const
	{
		counted = regions_[index].counted;
		uncounted = regions_[index].uncounted;
		return regions_[index].value;
	}

	unsigned int numRows_;
	unsigned int numCols_;
	unsigned int numFloating_;

	struct Region {
		T value;
		uint32_t counted;
		uint32_t uncounted;
	};

	std::vector<Region> regions_;
	T default_;
};

} /* namespace RPiController */
