/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * controller.cpp - ISP controller
 */

#include <assert.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "algorithm.h"
#include "controller.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiController)

static const std::map<std::string, Controller::HardwareConfig> HardwareConfigMap = {
	{
		"bcm2835",
		{
			/*
			* There are only ever 15 regions computed by the firmware
			* due to zoning, but the HW defines AGC_REGIONS == 16!
			*/			
			.agcRegions = { 15 , 1 },
			.agcZoneWeights = { 15 , 1 },
			.awbRegions = { 16, 12 },
			.focusRegions = { 4, 3 },
			.numHistogramBins = 128,
			.pipelineWidth = 13
		}
	},
	/* For error handling. */
	{
		"error",
		{
		}
	}
};

Controller::Controller()
	: switchModeCalled_(false)
{
}

Controller::~Controller() {}

int Controller::read(char const *filename)
{
	File file(filename);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(RPiController, Warning)
			<< "Failed to open tuning file '" << filename << "'";
		return -EINVAL;
	}

	std::unique_ptr<YamlObject> root = YamlParser::parse(file);
	double version = (*root)["version"].get<double>(1.0);
	target_ = (*root)["target"].get<std::string>("bcm2835");

	if (version < 2.0) {
		LOG(RPiController, Warning)
			<< "This format of the tuning file will be deprecated soon!"
			<< " Please use the convert_tuning.py utility to update to version 2.0.";

		for (auto const &[key, value] : root->asDict()) {
			int ret = createAlgorithm(key, value);
			if (ret)
				return ret;
		}
	} else if (version < 3.0) {
		if (!root->contains("algorithms")) {
			LOG(RPiController, Error)
				<< "Tuning file " << filename
				<< " does not have an \"algorithms\" list!";
			return -EINVAL;
		}

		for (auto const &rootAlgo : (*root)["algorithms"].asList())
			for (auto const &[key, value] : rootAlgo.asDict()) {
				int ret = createAlgorithm(key, value);
				if (ret)
					return ret;
			}
	} else {
		LOG(RPiController, Error)
			<< "Unrecognised version " << version
			<< " for the tuning file " << filename;
		return -EINVAL;
	}

	return 0;
}

int Controller::createAlgorithm(const std::string &name, const YamlObject &params)
{
	auto it = getAlgorithms().find(name);
	if (it == getAlgorithms().end()) {
		LOG(RPiController, Warning)
			<< "No algorithm found for \"" << name << "\"";
		return 0;
	}

	Algorithm *algo = (*it->second)(this);
	int ret = algo->read(params);
	if (ret)
		return ret;

	algorithms_.push_back(AlgorithmPtr(algo));
	return 0;
}

void Controller::initialise()
{
	for (auto &algo : algorithms_)
		algo->initialise();
}

void Controller::switchMode(CameraMode const &cameraMode, Metadata *metadata)
{
	for (auto &algo : algorithms_)
		algo->switchMode(cameraMode, metadata);
	switchModeCalled_ = true;
}

void Controller::prepare(Metadata *imageMetadata)
{
	assert(switchModeCalled_);
	for (auto &algo : algorithms_)
		algo->prepare(imageMetadata);
}

void Controller::process(StatisticsPtr stats, Metadata *imageMetadata)
{
	assert(switchModeCalled_);
	for (auto &algo : algorithms_)
		algo->process(stats, imageMetadata);
}

Metadata &Controller::getGlobalMetadata()
{
	return globalMetadata_;
}

Algorithm *Controller::getAlgorithm(std::string const &name) const
{
	/*
	 * The passed name must be the entire algorithm name, or must match the
	 * last part of it with a period (.) just before.
	 */
	size_t nameLen = name.length();
	for (auto &algo : algorithms_) {
		char const *algoName = algo->name();
		size_t algoNameLen = strlen(algoName);
		if (algoNameLen >= nameLen &&
		    strcasecmp(name.c_str(),
			       algoName + algoNameLen - nameLen) == 0 &&
		    (nameLen == algoNameLen ||
		     algoName[algoNameLen - nameLen - 1] == '.'))
			return algo.get();
	}
	return nullptr;
}

const std::string &Controller::getTarget() const
{
	return target_;
}

const Controller::HardwareConfig &Controller::getHardwareConfig() const
{
	auto cfg = HardwareConfigMap.find(getTarget());

	/*
	 * This really should not happen, the IPA ought to validate the target
	 * on initialisation.
	 */
	if (cfg == HardwareConfigMap.end())
		return HardwareConfigMap.at("error");

	return cfg->second;
}
