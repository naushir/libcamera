/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * delayed_controls.h - Helper to deal with controls that take effect with a delay
 */

#pragma once

#include <memory>
#include <unordered_map>

#include "libcamera/internal/delayed_sync_list.h"

namespace libcamera {

class V4L2Device;

class RPiDelayedControls
{
public:
	struct ControlParams {
		unsigned int delay;
		bool priorityWrite;
	};

	RPiDelayedControls(V4L2Device *device,
			   const std::unordered_map<unsigned int, ControlParams> &controlParams);

	void reset(std::unordered_map<unsigned int, std::any> &resetValues);

	bool push(std::unordered_map<unsigned int, std::any> &controls);
	std::unordered_map<unsigned int, std::any> get(uint32_t sequence);

	void applyControls(uint32_t sequence);

private:
	V4L2Device *device_;
	std::unique_ptr<DelayedSyncList<unsigned int>> delaySync_;
	std::unordered_map<unsigned int, ControlParams> controlParams_;
};

} /* namespace libcamera */
