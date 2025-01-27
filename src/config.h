#pragma once

// The threshold of acceleration along X, Y or Z axis required to start and
// finish registering a click, measured in milli-g (1/1000th of 9.81m/s^2).
// Must range from 0.5g to 7.5g with a step of 0.5g.
#define CONFIG_CLICK_THRESHOLD_MG 7'500

// The maximum time between the acceleration exceeding the threshold value and
// acceleration then falling below the threshold value required to register any
// click event. Must range from 0ms to 127.5ms with a step of 0.5ms.
#define CONFIG_CLICK_TIMELIMIT_US 127'500

// The minimal duration after which a double click event can be registered
// after a single click event. Must range from 0ms to 255ms with a step of 1ms.
#define CONFIG_CLICK_LATENCY_US 255'000

// The maximum time after the latency period during which a double click event
// can be start to be registered. The window does not limit when the double
// click registration finishes, as long as it finishes within the time limit.
// Must range from 0ms to 255ms with a step of 1ms.
#define CONFIG_CLICK_WINDOW_US 255'000
