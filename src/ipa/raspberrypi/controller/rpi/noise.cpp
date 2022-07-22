/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * noise.cpp - Noise control algorithm
 */

#include <math.h>

#include <libcamera/base/log.h>

#include "../device_status.h"
#include "../noise_status.h"

#include "noise.hpp"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiNoise)

#define NAME "rpi.noise"

Noise::Noise(Controller *controller)
	: Algorithm(controller), modeFactor_(1.0)
{
}

char const *Noise::name() const
{
	return NAME;
}

void Noise::switchMode(CameraMode const &cameraMode,
		       [[maybe_unused]] Metadata *metadata)
{
	// For example, we would expect a 2x2 binned mode to have a "noise
	// factor" of sqrt(2x2) = 2. (can't be less than one, right?)
	modeFactor_ = std::max(1.0, cameraMode.noiseFactor);
}

void Noise::read(boost::property_tree::ptree const &params)
{
	referenceConstant_ = params.get<double>("reference_constant");
	referenceSlope_ = params.get<double>("reference_slope");
}

void Noise::prepare(Metadata *imageMetadata)
{
	struct DeviceStatus deviceStatus;
	deviceStatus.analogueGain = 1.0; // keep compiler calm
	if (imageMetadata->get("device.status", deviceStatus) == 0) {
		// There is a slight question as to exactly how the noise
		// profile, specifically the constant part of it, scales. For
		// now we assume it all scales the same, and we'll revisit this
		// if it proves substantially wrong.  NOTE: we may also want to
		// make some adjustments based on the camera mode (such as
		// binning), if we knew how to discover it...
		double factor = sqrt(deviceStatus.analogueGain) / modeFactor_;
		struct NoiseStatus status;
		status.noise_constant = referenceConstant_ * factor;
		status.noise_slope = referenceSlope_ * factor;
		imageMetadata->set("noise.status", status);
		LOG(RPiNoise, Debug)
			<< "constant " << status.noise_constant
			<< " slope " << status.noise_slope;
	} else
		LOG(RPiNoise, Warning) << " no metadata";
}

// Register algorithm with the system.
static Algorithm *create(Controller *controller)
{
	return new Noise(controller);
}
static RegisterAlgorithm reg(NAME, &create);
