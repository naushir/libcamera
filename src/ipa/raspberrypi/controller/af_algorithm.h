/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * af_algorithm.hpp - auto focus algorithm interface
 */
#pragma once

#include <libcamera/base/span.h>

#include "algorithm.h"

namespace RPiController {

class AfAlgorithm : public Algorithm
{
public:
	AfAlgorithm(Controller *controller)
		: Algorithm(controller) {}

	/*
	 * An autofocus algorithm should provide the following calls.
	 *
	 * When setLensPosition() returns true, the hardware lens position
	 * should be forwarded immediately to the lens driver.
	 *
	 * This API is simpler and more permissive than libcamera's model:
	 * to emulate the libcamera interface, some controls must be ignored
	 * in certain modes. AfPause may be implemented by disabling CAF.
	 */

	enum AfRange { AfRangeNormal = 0,
		       AfRangeMacro,
		       AfRangeFull,
		       AfRangeMax };

	enum AfSpeed { AfSpeedNormal = 0,
		       AfSpeedFast,
		       AfSpeedMax };

	virtual void setRange([[maybe_unused]] AfRange range)
	{
	}
	virtual void setSpeed([[maybe_unused]] AfSpeed speed)
	{
	}
	virtual void setMetering([[maybe_unused]] bool use_windows)
	{
	}
	virtual void setWindows([[maybe_unused]] libcamera::Span<libcamera::Rectangle const> const &wins)
	{
	}

	virtual bool setLensPosition(double dioptres, int32_t *hwpos) = 0;
	virtual void enableCAF(bool enable) = 0;
	virtual void triggerScan() = 0;
	virtual void cancelScan() = 0;
};

} // namespace RPiController
