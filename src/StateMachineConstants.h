#pragma once

#include <cstdint>
constexpr float PRELAUNCH_TO_BOOST_MIN_ACCEL = 5 * 9.81; // m/s^2

constexpr float BOOST_TO_COST_ACCEL_EPSILON = 0.3f; // m/s^2
constexpr uint32_t MAX_MOTOR_BURN_TIME = 2000; // ms
