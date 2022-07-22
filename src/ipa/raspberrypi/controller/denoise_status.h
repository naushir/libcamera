/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2022, Raspberry Pi Ltd
 *
 * denoise_status.h - Denoise control algorithm status
 */
#pragma once

/* This stores the parameters required for Denoise. */

struct DenoiseStatus {
	double noise_constant;
	double noise_slope;
	double strength;
	unsigned int mode;
};
