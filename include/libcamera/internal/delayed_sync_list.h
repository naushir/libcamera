/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * delayed_sync_list.h - Helper to deal with controls that take effect with a delay
 */

#pragma once

#include <stdint.h>

#include <any>
#include <unordered_map>

#include <libcamera/controls.h>

namespace libcamera {

template<typename T>
class DelayedSyncList
{
public:
	DelayedSyncList(const std::unordered_map<T, unsigned int> &delayParams)
		: maxDelay_(0)
	{
		/*
                * Create a map of control ids to delays for controls exposed by the
                * device.
                */
		for (auto const &[k, delay] : delayParams) {
			delays_[k] = delay;
			maxDelay_ = std::max(maxDelay_, delay);
		}

		reset({});
	}

	void reset(const std::unordered_map<T, std::any> &defaultValues)
	{
		values_.clear();
		for (const auto &[key, value] : defaultValues)
			values_[key][0] = Value(value, false);

		queueCount_ = defaultValues.empty() ? 0 : 1;
		writeCount_ = 0;
	}

	bool push(const std::unordered_map<T, std::any> &items)
	{
		/* Copy state from previous frame. */
		for (auto &[key, values] : values_) {
			Value &value = values[queueCount_];
			value = values_[key][queueCount_ - 1];
			value.updated = false;
		}

		/* Update with new controls. */
		for (const auto &[key, value] : items)
			values_[key][queueCount_] = Value(value);

		queueCount_++;
		return true;
	}

	std::unordered_map<T, std::any> get(uint32_t sequence)
	{
		unsigned int index = std::max<int>(0, sequence - maxDelay_);

		std::unordered_map<T, std::any> out;
		for (const auto &[key, values] : values_)
			out.emplace(key, values[index].value);

		return out;
	}

	std::unordered_map<T, std::any> nextFrame(uint32_t sequence)
	{
		std::unordered_map<T, std::any> out;

		for (auto &[key, values] : values_) {
			unsigned int delayDiff = maxDelay_ - delays_[key];
			unsigned int index = std::max<int>(0, writeCount_ - delayDiff);

			if (values[index].updated) {
				out.emplace(key, values[index].value);
				/* Done with this update, so mark as completed. */
				values[index].updated = false;
			}
		}

		writeCount_ = sequence + 1;

		while (writeCount_ > queueCount_)
			push({});

		return out;
	}

private:
	struct Value
        {
		Value()
			: updated(false)
		{
		}

		Value(const std::any &v, bool updated_ = true)
			: value(v), updated(updated_)
		{
		}

		std::any value;
		bool updated;
	};

	static constexpr int listSize = 16;
	class DelayRingBuffer : public std::array<Value, listSize>
	{
	public:
		Value &operator[](unsigned int index)
		{
			return std::array<Value, listSize>::operator[](index % listSize);
		}

		const Value &operator[](unsigned int index) const
		{
			return std::array<Value, listSize>::operator[](index % listSize);
		}
	};

	std::unordered_map<T, unsigned int> delays_;
	unsigned int maxDelay_;

	uint32_t queueCount_;
	uint32_t writeCount_;
	std::unordered_map<T, DelayRingBuffer> values_;
};

} /* namespace libcamera */
