/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2022, Raspberry Pi Ltd
 *
 * alsc_status.h - ALSC (auto lens shading correction) control algorithm status
 */
#pragma once

/*
 * The ALSC algorithm should post the following structure into the image's
 * "alsc.status" metadata.
 */

#define ALSC_CELLS_X 16
#define ALSC_CELLS_Y 12

struct AlscStatus {
	double r[ALSC_CELLS_Y][ALSC_CELLS_X];
	double g[ALSC_CELLS_Y][ALSC_CELLS_X];
	double b[ALSC_CELLS_Y][ALSC_CELLS_X];
};
