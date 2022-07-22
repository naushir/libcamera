/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2022, Raspberry Pi Ltd
 *
 * sharpen.h - sharpening control algorithm
 */
#pragma once

#include "../sharpen_algorithm.h"
#include "../sharpen_status.h"

/* This is our implementation of the "sharpen algorithm". */

namespace RPiController {

class Sharpen : public SharpenAlgorithm
{
public:
	Sharpen(Controller *controller);
	char const *name() const override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void read(boost::property_tree::ptree const &params) override;
	void setStrength(double strength) override;
	void prepare(Metadata *imageMetadata) override;

private:
	double threshold_;
	double strength_;
	double limit_;
	double modeFactor_;
	double userStrength_;
};

} /* namespace RPiController */
