/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * delayed_controls.h - Helper to deal with controls that take effect with a delay
 */

#pragma once

#include <stdint.h>
#include <memory>
#include <unordered_map>

#include "delayed_sync_list.h"

#include <libcamera/controls.h>

namespace libcamera {

class V4L2Device;

class DelayedControls
{
public:
	struct ControlParams {
		unsigned int delay;
		bool priorityWrite;
	};

	DelayedControls(V4L2Device *device,
			const std::unordered_map<uint32_t, ControlParams> &controlParams);

	void reset();

	bool push(const ControlList &controls);
	ControlList get(uint32_t sequence);

	void applyControls(uint32_t sequence);

private:
	V4L2Device *device_;
	std::unique_ptr<DelayedSyncList<unsigned int>> delaySync_;
	std::unordered_map<unsigned int, ControlParams> controlParams_;
};

} /* namespace libcamera */
