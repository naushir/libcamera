/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * agc.hpp - AGC/AEC control algorithm
 */
#pragma once

#include <vector>
#include <mutex>

#include <libcamera/base/utils.h>

#include "../agc_algorithm.hpp"
#include "../agc_status.h"
#include "../pwl.hpp"

/* This is our implementation of AGC. */

/*
 * This is the number actually set up by the firmware, not the maximum possible
 * number (which is 16).
 */

#define AGC_STATS_SIZE 15

namespace RPiController {

struct AgcMeteringMode {
	double weights[AGC_STATS_SIZE];
	void read(boost::property_tree::ptree const &params);
};

struct AgcExposureMode {
	std::vector<libcamera::utils::Duration> shutter;
	std::vector<double> gain;
	void read(boost::property_tree::ptree const &params);
};

struct AgcConstraint {
	enum class Bound { LOWER = 0, UPPER = 1 };
	Bound bound;
	double qLo;
	double qHi;
	Pwl yTarget;
	void read(boost::property_tree::ptree const &params);
};

typedef std::vector<AgcConstraint> AgcConstraintMode;

struct AgcConfig {
	void read(boost::property_tree::ptree const &params);
	std::map<std::string, AgcMeteringMode> meteringModes;
	std::map<std::string, AgcExposureMode> exposureModes;
	std::map<std::string, AgcConstraintMode> constraintModes;
	Pwl yTarget;
	double speed;
	uint16_t startupFrames;
	unsigned int convergenceFrames;
	double maxChange;
	double minChange;
	double fastReduceThreshold;
	double speedUpThreshold;
	std::string defaultMeteringMode;
	std::string defaultExposureMode;
	std::string defaultConstraintMode;
	double baseEv;
	libcamera::utils::Duration defaultExposureTime;
	double defaultAnalogueGain;
};

class Agc : public AgcAlgorithm
{
public:
	Agc(Controller *controller);
	char const *name() const override;
	void read(boost::property_tree::ptree const &params) override;
	/* AGC handles "pausing" for itself. */
	bool isPaused() const override;
	void pause() override;
	void resume() override;
	unsigned int getConvergenceFrames() const override;
	void setEv(double ev) override;
	void setFlickerPeriod(libcamera::utils::Duration flickerPeriod) override;
	void setMaxShutter(libcamera::utils::Duration maxShutter) override;
	void setFixedShutter(libcamera::utils::Duration fixedShutter) override;
	void setFixedAnalogueGain(double fixedAnalogueGain) override;
	void setMeteringMode(std::string const &meteringModeName) override;
	void setExposureMode(std::string const &exposureModeName) override;
	void setConstraintMode(std::string const &contraintModeName) override;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;

private:
	void updateLockStatus(DeviceStatus const &deviceStatus);
	AgcConfig config_;
	void housekeepConfig();
	void fetchCurrentExposure(Metadata *imageMetadata);
	void fetchAwbStatus(Metadata *imageMetadata);
	void computeGain(bcm2835_isp_stats *statistics, Metadata *imageMetadata,
			 double &gain, double &targetY);
	void computeTargetExposure(double gain);
	bool applyDigitalGain(double gain, double targetY);
	void filterExposure(bool desaturate);
	void divideUpExposure();
	void writeAndFinish(Metadata *imageMetadata, bool desaturate);
	libcamera::utils::Duration clipShutter(libcamera::utils::Duration shutter);
	AgcMeteringMode *meteringMode_;
	AgcExposureMode *exposureMode_;
	AgcConstraintMode *constraintMode_;
	uint64_t frameCount_;
	AwbStatus awb_;
	struct ExposureValues {
		ExposureValues();

		libcamera::utils::Duration shutter;
		double analogueGain;
		libcamera::utils::Duration totalExposure;
		libcamera::utils::Duration totalExposureNoDG; /* without digital gain */
	};
	ExposureValues current_;  /* values for the current frame */
	ExposureValues target_;   /* calculate the values we want here */
	ExposureValues filtered_; /* these values are filtered towards target */
	AgcStatus status_;
	int lockCount_;
	DeviceStatus lastDeviceStatus_;
	libcamera::utils::Duration lastTargetExposure_;
	double lastSensitivity_; /* sensitivity of the previous camera mode */
	/* Below here the "settings" that applications can change. */
	std::string meteringModeName_;
	std::string exposureModeName_;
	std::string constraintModeName_;
	double ev_;
	libcamera::utils::Duration flickerPeriod_;
	libcamera::utils::Duration maxShutter_;
	libcamera::utils::Duration fixedShutter_;
	double fixedAnalogueGain_;
};

} /* namespace RPiController */
