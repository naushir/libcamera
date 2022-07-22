/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2022, Raspberry Pi Ltd
 *
 * sdn.cpp - SDN (spatial denoise) control algorithm
 */

#include <libcamera/base/log.h>

#include "../denoise_status.h"
#include "../noise_status.h"

#include "sdn.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiSdn)

/*
 * Calculate settings for the spatial denoise block using the noise profile in
 * the image metadata.
 */

#define NAME "rpi.sdn"

Sdn::Sdn(Controller *controller)
	: DenoiseAlgorithm(controller), mode_(DenoiseMode::ColourOff)
{
}

char const *Sdn::name() const
{
	return NAME;
}

void Sdn::read(boost::property_tree::ptree const &params)
{
	deviation_ = params.get<double>("deviation", 3.2);
	strength_ = params.get<double>("strength", 0.75);
}

void Sdn::initialise()
{
}

void Sdn::prepare(Metadata *imageMetadata)
{
	struct NoiseStatus noiseStatus = {};
	noiseStatus.noise_slope = 3.0; /* in case no metadata */
	if (imageMetadata->get("noise.status", noiseStatus) != 0)
		LOG(RPiSdn, Warning) << "no noise profile found";
	LOG(RPiSdn, Debug)
		<< "Noise profile: constant " << noiseStatus.noise_constant
		<< " slope " << noiseStatus.noise_slope;
	struct DenoiseStatus status;
	status.noise_constant = noiseStatus.noise_constant * deviation_;
	status.noise_slope = noiseStatus.noise_slope * deviation_;
	status.strength = strength_;
	status.mode = static_cast<std::underlying_type_t<DenoiseMode>>(mode_);
	imageMetadata->set("denoise.status", status);
	LOG(RPiSdn, Debug)
		<< "programmed constant " << status.noise_constant
		<< " slope " << status.noise_slope
		<< " strength " << status.strength;
}

void Sdn::setMode(DenoiseMode mode)
{
	/* We only distinguish between off and all other modes. */
	mode_ = mode;
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new Sdn(controller);
}
static RegisterAlgorithm reg(NAME, &create);
