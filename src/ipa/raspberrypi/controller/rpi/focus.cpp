/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * focus.cpp - focus algorithm
 */
#include <stdint.h>

#include <libcamera/base/log.h>

#include "focus.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiFocus)

#define NAME "rpi.focus"

Focus::Focus(Controller *controller)
	: Algorithm(controller)
{
}

char const *Focus::name() const
{
	return NAME;
}

void Focus::process(StatisticsPtr &stats, Metadata *imageMetadata)
{
	/* Pass the stats directly to the IPA to report out to the application */
	imageMetadata->set("focus.status", stats->focusRegions);
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return new Focus(controller);
}
static RegisterAlgorithm reg(NAME, &create);
