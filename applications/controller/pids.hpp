#ifndef PIDS_HPP
#define PIDS_HPP
#include "HERO_SELECTION.hpp"
#include "tools/pid/pid.hpp"

inline sp::PID speed_lf_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.5f, 0.15f);
inline sp::PID speed_lr_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.8f, 0.15f);
inline sp::PID speed_rf_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.5f, 0.15f);
inline sp::PID speed_rr_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.5f, 0.15f);

#endif  // PIDS_HPP