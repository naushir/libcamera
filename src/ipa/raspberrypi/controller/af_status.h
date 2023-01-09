/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * af_status.h - AF control algorithm status
 */
#pragma once

/*
 * The AF algorithm should post the following structure into the image's
 * "af.status" metadata. lensDriven and lensSetting should control the lens.
 */
enum class AfState {
	/* Initial state before trigger() or enableCAF() */
	Unknown,
	/* Scan in progress or CAF large lens movement */
	Scanning,
	/* Scan succeeded or CAF has (nearly) converged */
	Focused,
	/* Scan failed or CAF lens position out of range */
	Failed
};

struct AfStatus {
	AfState state;
	/* lensEstimate gives an estimate of focus in dioptres */
	bool lensKnown;
	/* lensSetting should be sent to the lens driver */
	bool lensDriven;
	/* Estimated current lens position in dioptres */
	float lensEstimate;
	/* Desired new lens position in HW units */
	int32_t lensSetting;
};
