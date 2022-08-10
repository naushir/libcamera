/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * delayed_controls.h - Helper to deal with controls that take effect with a delay
 */
#include "rpi_delayed_controls.h"

#include <libcamera/base/log.h>

#include <libcamera/controls.h>

#include "libcamera/internal/v4l2_device.h"

/**
 * \file delayed_controls.h
 * \brief Helper to deal with controls that take effect with a delay
 */

namespace libcamera {

/**
 * \class RPiDelayedControls
 * \brief Helper to deal with controls that take effect with a delay
 *
 * Some sensor controls take effect with a delay as the sensor needs time to
 * adjust, for example exposure and analog gain. This is a helper class to deal
 * with such controls and the intended users are pipeline handlers.
 *
 * The idea is to extend the concept of the buffer depth of a pipeline the
 * application needs to maintain to also cover controls. Just as with buffer
 * depth if the application keeps the number of requests queued above the
 * control depth the controls are guaranteed to take effect for the correct
 * request. The control depth is determined by the control with the greatest
 * delay.
 */

/**
 * \struct RPiDelayedControls::ControlParams
 * \brief Parameters associated with controls handled by the \a RPiDelayedControls
 * helper class
 *
 * \var ControlParams::delay
 * \brief Frame delay from setting the control on a sensor device to when it is
 * consumed during framing.
 *
 * \var ControlParams::priorityWrite
 * \brief Flag to indicate that this control must be applied ahead of, and
 * separately from the other controls.
 *
 * Typically set for the \a V4L2_CID_VBLANK control so that the device driver
 * does not reject \a V4L2_CID_EXPOSURE control values that may be outside of
 * the existing vertical blanking specified bounds, but are within the new
 * blanking bounds.
 */

/**
 * \brief Construct a RPiDelayedControls instance
 * \param[in] device The V4L2 device the controls have to be applied to
 * \param[in] controlParams Map of the numerical V4L2 control ids to their
 * associated control parameters.
 *
 * The control parameters comprise of delays (in frames) and a priority write
 * flag. If this flag is set, the relevant control is written separately from,
 * and ahead of the rest of the batched controls.
 *
 * Only controls specified in \a controlParams are handled. If it's desired to
 * mix delayed controls and controls that take effect immediately the immediate
 * controls must be listed in the \a controlParams map with a delay value of 0.
 */
RPiDelayedControls::RPiDelayedControls(V4L2Device *device,
				       const std::unordered_map<uint32_t, ControlParams> &controlParams)
	: device_(device)
{
	std::unordered_map<unsigned int, unsigned int> delayParams;

	/*
	 * Create a map of control ids to delays for controls exposed by the
	 * device.
	 */
	for (auto const &[id, param] : controlParams) {
		controlParams_[id] = param;
		delayParams[id] = param.delay;
	}

	delaySync_ = std::make_unique<DelayedSyncList<unsigned int>>(delayParams);
}

/**
 * \brief Reset state machine
 *
 * Resets the state machine to a starting position based on control values
 * retrieved from the device.
 */
void RPiDelayedControls::reset(std::unordered_map<unsigned int, std::any> &resetValues)
{
	/* Retrieve control as reported by the device. */
	const ControlInfoMap &controls = device_->controls();
	std::vector<uint32_t> ids;
	for (auto const &[id, param] : controlParams_) {
		/* Not a device control. */
		if (controls.find(id) == controls.end())
			continue;

		ids.push_back(id);
	}

	ControlList deviceControls = device_->getControls(ids);

	/* Seed the control queue with the controls reported by the device. */
	for (const auto &ctrl : deviceControls)
		resetValues.emplace(ctrl.first, ctrl.second);

	delaySync_->reset(resetValues);
}

/**
 * \brief Push a set of controls on the queue
 * \param[in] controls List of controls to add to the device queue
 *
 * Push a set of controls to the control queue. This increases the control queue
 * depth by one.
 *
 * \returns true if \a controls are accepted, or false otherwise
 */
bool RPiDelayedControls::push(std::unordered_map<unsigned int, std::any> &controls)
{
	delaySync_->push(controls);
	return true;
}

/**
 * \brief Read back controls in effect at a sequence number
 * \param[in] sequence The sequence number to get controls for
 *
 * Read back what controls where in effect at a specific sequence number. The
 * history is a ring buffer of 16 entries where new and old values coexist. It's
 * the callers responsibility to not read too old sequence numbers that have been
 * pushed out of the history.
 *
 * Historic values are evicted by pushing new values onto the queue using
 * push(). The max history from the current sequence number that yields valid
 * values are thus 16 minus number of controls pushed.
 *
 * \return The controls at \a sequence number
 */
std::unordered_map<unsigned int, std::any> RPiDelayedControls::get(uint32_t sequence)
{
	return delaySync_->get(sequence);
}

/**
 * \brief Inform RPiDelayedControls of the start of a new frame
 * \param[in] sequence Sequence number of the frame that started
 *
 * Inform the state machine that a new frame has started and of its sequence
 * number. Any user of these helpers is responsible to inform the helper about
 * the start of any frame. This can be connected with ease to the start of a
 * exposure (SOE) V4L2 event.
 */
void RPiDelayedControls::applyControls(uint32_t sequence)
{
	auto ctrls = delaySync_->nextFrame(sequence);
	/*
	 * Create control list peeking ahead in the value queue to ensure
	 * values are set in time to satisfy the sensor delay.
	 */
	const ControlInfoMap &controls = device_->controls();
	ControlList out(controls);
	for (auto &[id, value] : ctrls) {
		/* Not a device control. */
		if (controls.find(id) == controls.end())
			continue;

		if (controlParams_[id].priorityWrite) {
			/*
			 * This control must be written now, it could
			 * affect validity of the other controls.
			 */
			ControlList priority(device_->controls());
			priority.set(id, std::any_cast<ControlValue>(value));
			device_->setControls(&priority);
		} else {
			/*
			 * Batch up the list of controls and write them
			 * at the end of the function.
			 */
			out.set(id, std::any_cast<ControlValue>(value));
		}
	}

	device_->setControls(&out);
}

} /* namespace libcamera */
