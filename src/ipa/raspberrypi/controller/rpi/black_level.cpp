/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * black_level.cpp - black level control algorithm
 */

#include <math.h>
#include <stdint.h>

#include <libcamera/base/log.h>

#include "../black_level_status.h"

#include "black_level.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiBlackLevel)

#define NAME "rpi.black_level"

BlackLevel::BlackLevel(Controller *controller)
	: Algorithm(controller)
{
}

char const *BlackLevel::name() const
{
	return NAME;
}

void BlackLevel::read(boost::property_tree::ptree const &params)
{
	uint16_t blackLevel = params.get<uint16_t>(
		"black_level", 4096); /* 64 in 10 bits scaled to 16 bits */
	blackLevelR_ = params.get<uint16_t>("black_level_r", blackLevel);
	blackLevelG_ = params.get<uint16_t>("black_level_g", blackLevel);
	blackLevelB_ = params.get<uint16_t>("black_level_b", blackLevel);
	LOG(RPiBlackLevel, Debug)
		<< " Read black levels red " << blackLevelR_
		<< " green " << blackLevelG_
		<< " blue " << blackLevelB_;
}

void BlackLevel::prepare(Metadata *imageMetadata)
{
	/*
	 * Possibly we should think about doing this in a switch_mode or
	 * something?
	 */
	struct BlackLevelStatus status;
	status.black_level_r = blackLevelR_;
	status.black_level_g = blackLevelG_;
	status.black_level_b = blackLevelB_;
	imageMetadata->set("black_level.status", status);
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return new BlackLevel(controller);
}
static RegisterAlgorithm reg(NAME, &create);
