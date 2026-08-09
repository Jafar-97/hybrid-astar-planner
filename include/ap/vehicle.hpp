#pragma once
// Kinematic bicycle model. This is the motion model shared by the planner
// (for node expansion) and the simulator (for closed-loop tracking), so the
// path the planner produces is guaranteed to be dynamically feasible for the
// same vehicle that drives it.
#include "ap/common.hpp"

namespace ap {

struct VehicleParams {
    double wheelbase = 2.7;      // L, front-to-rear axle distance [m]
    double max_steer = 0.55;     // max |steering angle| [rad] (~31.5 deg)
    double width = 1.9;          // vehicle width [m]
    double length = 4.5;         // vehicle length [m]
    double max_speed = 8.0;      // [m/s]
    double max_accel = 2.5;      // [m/s^2]

    // Minimum turning radius implied by the geometry: R = L / tan(max_steer).
    double minTurnRadius() const { return wheelbase / std::tan(max_steer); }
    // Radius of a circle that bounds the footprint, used for fast collision checks.
    double collisionRadius() const {
        double hl = 0.5 * length, hw = 0.5 * width;
        return hypot2(hl, hw);
    }
};

// Integrate the bicycle model one step of arc-length ds (metres) at a fixed
// steering angle. Positive ds drives forward. Uses the exact kinematic update.
inline Pose2D bicycleStep(const Pose2D& p, double steer, double ds, double L) {
    Pose2D n = p;
    n.x = p.x + ds * std::cos(p.theta);
    n.y = p.y + ds * std::sin(p.theta);
    n.theta = normalizeAngle(p.theta + ds * std::tan(steer) / L);
    return n;
}

}  // namespace ap
