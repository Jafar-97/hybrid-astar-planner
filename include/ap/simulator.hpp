#pragma once
// Closed-loop simulator: drives the bicycle-model vehicle along a planned path
// using the pure-pursuit controller and records the executed trajectory plus
// tracking-quality metrics.
#include "ap/common.hpp"
#include "ap/controller.hpp"
#include "ap/vehicle.hpp"
#include <vector>

namespace ap {

struct TrajSample {
    double t, x, y, theta, v, steer, target_x, target_y, cte;
};

struct TrackingMetrics {
    bool completed = false;
    double sim_time = 0.0;
    double cte_rms = 0.0;
    double cte_max = 0.0;
    double avg_speed = 0.0;
    double distance = 0.0;
};

struct SimResult {
    std::vector<TrajSample> log;
    TrackingMetrics metrics;
};

SimResult simulate(const std::vector<Pose2D>& path, const Pose2D& start,
                   const VehicleParams& veh, const ControllerConfig& ccfg,
                   double dt = 0.05, double max_time = 90.0);

}  // namespace ap
