/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * contrast.hpp - contrast (gamma) control algorithm
 */
#pragma once

#include <mutex>

#include "../contrast_algorithm.hpp"
#include "../pwl.hpp"

namespace RPiController {

/*
 * Back End algorithm to appaly correct digital gain. Should be placed after
 * Back End AWB.
 */

struct ContrastConfig {
	bool ceEnable;
	double loHistogram;
	double loLevel;
	double loMax;
	double hiHistogram;
	double hiLevel;
	double hiMax;
	Pwl gammaCurve;
};

class Contrast : public ContrastAlgorithm
{
public:
	Contrast(Controller *controller = NULL);
	char const *name() const override;
	void read(boost::property_tree::ptree const &params) override;
	void setBrightness(double brightness) override;
	void setContrast(double contrast) override;
	void initialise() override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;

private:
	ContrastConfig config_;
	double brightness_;
	double contrast_;
	ContrastStatus status_;
	std::mutex mutex_;
};

} /* namespace RPiController */
