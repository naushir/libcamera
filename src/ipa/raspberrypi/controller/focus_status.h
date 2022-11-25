/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * focus_status.h - focus measurement status
 */
#pragma once

#include <vector>

/*
 * The focus algorithm should post the following structure into the image's
 * "focus.status" metadata. Recall that it's only reporting focus (contrast)
 * measurements, it's not driving any kind of auto-focus algorithm!
 */

struct FocusStatus {
	std::vector<uint32_t> focusMeasures;
};
