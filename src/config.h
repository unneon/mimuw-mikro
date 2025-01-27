#pragma once

// The minimal acceleration along X, Y or Z axis required to register a click,
// measured in milli-g (1/1000th of 9.81m/s^2). Must range from 0.5g to 7.5g
// with a step of 0.5g.
#define CONFIG_CLICK_THRESHOLD_MG 500

// TODO: Document how do these options exactly work?

// "Time limit" between clicks, as (not) defined by LIS35DE documentation. Must
// range from 0ms to 127.5ms with a step of 0.5ms.
#define CONFIG_CLICK_TIMELIMIT_US 127'500

// "Latency" between clicks, as (not) defined by LIS35DE documentation. Must
// range from 0ms to 255ms with a step of 1ms.
#define CONFIG_CLICK_LATENCY_US 255'000

// "Window" between clicks, as (not) defined by LIS35DE documentation. Must
// range from 0ms to 255ms with a step of 1ms.
#define CONFIG_CLICK_WINDOW_US 255'000
