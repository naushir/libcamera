/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2022, Raspberry Pi Ltd
 *
 * noise.h - Noise control algorithm
 */
#pragma once

#include "../algorithm.h"
#include "../noise_status.h"

/* This is our implementation of the "noise algorithm". */

namespace RPiController {

class Noise : public Algorithm
{
public:
	Noise(Controller *controller);
	char const *name() const override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void read(boost::property_tree::ptree const &params) override;
	void prepare(Metadata *imageMetadata) override;

private:
	/* the noise profile for analogue gain of 1.0 */
	double referenceConstant_;
	double referenceSlope_;
	double modeFactor_;
};

} /* namespace RPiController */
