#pragma once
// Path-tracking controller: pure-pursuit for steering (lateral) and a PID for
// speed (longitudinal). Pure pursuit picks a look-ahead point on the reference
// path and geometrically solves for the steering angle that arcs the vehicle
// onto it; the look-ahead distance grows with speed for stability.
#include "ap/common.hpp"
#include "ap/vehicle.hpp"
#include <vector>

namespace ap {

struct ControllerConfig {
    double lookahead_min = 2.0;   // [m] short enough to track near-min-radius arcs
    double lookahead_k = 0.3;     // look-ahead gain on speed [s]
    double target_speed = 4.0;    // [m/s]
    double curve_slowdown = 3.5;  // speed reduction factor per rad of steering
    double kp = 1.2, ki = 0.05, kd = 0.05;  // speed PID gains
    double goal_tol = 1.0;        // [m]
};

struct ControlCommand {
    double steer = 0.0;    // [rad]
    double accel = 0.0;    // [m/s^2]
    double target_x = 0.0; // look-ahead point (for logging/viz)
    double target_y = 0.0;
    double cross_track = 0.0;
    bool finished = false;
};

class PurePursuitController {
public:
    PurePursuitController(std::vector<Pose2D> path, const VehicleParams& veh,
                          ControllerConfig cfg = {})
        : path_(std::move(path)), veh_(veh), cfg_(cfg) {}

    // Compute a command for the current pose and speed.
    ControlCommand control(const Pose2D& pose, double speed, double dt);

private:
    std::vector<Pose2D> path_;
    VehicleParams veh_;
    ControllerConfig cfg_;
    size_t last_idx_ = 0;
    double integral_ = 0.0;
    double prev_err_ = 0.0;

    size_t nearestIndex(const Pose2D& pose) const;
};

}  // namespace ap
