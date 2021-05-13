/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * duration.hpp - Helper macros for time duration handling.
 */
#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>

namespace RPiController {

using Duration = std::chrono::duration<double, std::nano>;

// Helper to convert and return the count of any std::chrono::duration type to
// another timebase.  Use a double rep type to try and preserve precision.
template <typename P, typename T>
static constexpr double DurationValue(T const &d)
{
	return std::chrono::duration_cast<std::chrono::duration<double, P>>(d).count();
};

static inline std::ostream &operator<<(std::ostream &os, const Duration &duration)
{
        std::streamsize ss = os.precision();
        os << std::fixed << std::setprecision(2) << DurationValue<std::micro>(duration) << " us";
        os << std::setprecision(ss) << std::defaultfloat;
        return os;
}

} // namespace RPiController