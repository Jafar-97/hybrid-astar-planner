#include "ap/controller.hpp"
#include <algorithm>
#include <cmath>

namespace ap {

size_t PurePursuitController::nearestIndex(const Pose2D& pose) const {
    // Search forward from the last index so tracking progresses monotonically.
    size_t best = last_idx_;
    double best_d = 1e18;
    for (size_t i = last_idx_; i < path_.size(); ++i) {
        double d = dist(pose, path_[i]);
        if (d < best_d) {
            best_d = d;
            best = i;
        }
        if (d > best_d + 5.0) break;  // moved clearly past the closest point
    }
    return best;
}

ControlCommand PurePursuitController::control(const Pose2D& pose, double speed,
                                              double dt) {
    ControlCommand cmd;
    if (path_.empty()) {
        cmd.finished = true;
        return cmd;
    }

    size_t ni = nearestIndex(pose);
    last_idx_ = ni;

    // Cross-track error: signed lateral distance to the nearest path point.
    {
        const Pose2D& r = path_[ni];
        double dx = pose.x - r.x, dy = pose.y - r.y;
        cmd.cross_track = -std::sin(r.theta) * dx + std::cos(r.theta) * dy;
    }

    // Finished when close to the final waypoint.
    if (dist(pose, path_.back()) < cfg_.goal_tol) {
        cmd.finished = true;
        cmd.accel = -veh_.max_accel;  // brake
        return cmd;
    }

    // Look-ahead point: walk forward until we exceed the look-ahead distance.
    double Ld = cfg_.lookahead_min + cfg_.lookahead_k * std::max(0.0, speed);
    size_t ti = ni;
    for (size_t i = ni; i < path_.size(); ++i) {
        ti = i;
        if (dist(pose, path_[i]) >= Ld) break;
    }
    const Pose2D& target = path_[ti];
    cmd.target_x = target.x;
    cmd.target_y = target.y;

    // Pure-pursuit steering law.
    double dx = target.x - pose.x, dy = target.y - pose.y;
    double alpha = normalizeAngle(std::atan2(dy, dx) - pose.theta);
    double ld = std::max(dist(pose, target), 1e-3);
    double steer = std::atan2(2.0 * veh_.wheelbase * std::sin(alpha), ld);
    cmd.steer = std::clamp(steer, -veh_.max_steer, veh_.max_steer);

    // Longitudinal PID toward target speed; ease off in sharp turns.
    double curve_factor = 1.0 / (1.0 + cfg_.curve_slowdown * std::fabs(cmd.steer));
    double v_ref = cfg_.target_speed * curve_factor;
    double err = v_ref - speed;
    integral_ += err * dt;
    double deriv = (err - prev_err_) / std::max(dt, 1e-6);
    prev_err_ = err;
    double accel = cfg_.kp * err + cfg_.ki * integral_ + cfg_.kd * deriv;
    cmd.accel = std::clamp(accel, -veh_.max_accel, veh_.max_accel);
    return cmd;
}

}  // namespace ap
