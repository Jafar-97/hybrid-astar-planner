#include "ap/simulator.hpp"
#include <algorithm>
#include <cmath>

namespace ap {

SimResult simulate(const std::vector<Pose2D>& path, const Pose2D& start,
                   const VehicleParams& veh, const ControllerConfig& ccfg,
                   double dt, double max_time) {
    SimResult res;
    if (path.empty()) return res;

    PurePursuitController ctrl(path, veh, ccfg);
    Pose2D pose = start;
    double v = 0.0;
    double t = 0.0;
    double sum_sq_cte = 0.0, max_cte = 0.0, sum_speed = 0.0, dist_traveled = 0.0;
    int n = 0;
    Pose2D prev = pose;

    while (t < max_time) {
        ControlCommand cmd = ctrl.control(pose, v, dt);
        double cte = std::fabs(cmd.cross_track);
        sum_sq_cte += cte * cte;
        max_cte = std::max(max_cte, cte);
        sum_speed += v;
        ++n;

        res.log.push_back({t, pose.x, pose.y, pose.theta, v, cmd.steer,
                           cmd.target_x, cmd.target_y, cmd.cross_track});

        if (cmd.finished && v < 0.3) {
            res.metrics.completed = true;
            break;
        }
        // Safety abort if the controller has clearly lost the path.
        if (cte > 6.0) break;

        // Integrate longitudinal + lateral dynamics.
        v += cmd.accel * dt;
        v = std::clamp(v, 0.0, veh.max_speed);
        pose = bicycleStep(pose, cmd.steer, v * dt, veh.wheelbase);

        dist_traveled += dist(prev, pose);
        prev = pose;
        t += dt;
    }

    res.metrics.sim_time = t;
    res.metrics.cte_rms = (n > 0) ? std::sqrt(sum_sq_cte / n) : 0.0;
    res.metrics.cte_max = max_cte;
    res.metrics.avg_speed = (n > 0) ? sum_speed / n : 0.0;
    res.metrics.distance = dist_traveled;
    return res;
}

}  // namespace ap
