#pragma once
// Hybrid A* motion planner.
//
// Unlike grid A*, which searches over cell centres and produces jagged,
// kinematically-infeasible paths, Hybrid A* searches over *continuous* vehicle
// states (x, y, heading) that are reachable by integrating the bicycle model
// under a discrete set of steering actions. States are only discretised for the
// purpose of the closed set. Two ideas keep it tractable:
//
//   * A dual heuristic h = max(h_nonholo, h_holo):
//       - h_nonholo: the Dubins distance to the goal (respects the car's
//         turning radius but ignores obstacles).
//       - h_holo: a Dijkstra field over the costmap from the goal (respects
//         obstacles but ignores heading).
//     Taking the max of the two is admissible and dramatically prunes the tree.
//
//   * Analytic (Dubins) expansion: periodically we try to connect the current
//     node straight to the goal with a Dubins curve; if it is collision-free we
//     finish exactly on the goal pose instead of relying on lucky discretisation.
#include "ap/common.hpp"
#include "ap/grid_map.hpp"
#include "ap/vehicle.hpp"
#include <vector>

namespace ap {

struct PlannerConfig {
    double xy_resolution = 0.5;     // closed-set grid cell [m]
    int theta_bins = 72;            // heading discretisation (5 deg)
    double step_size = 1.4;         // arc length per expansion [m]
    int n_steer = 5;                // steering samples in [-max, max]
    double steer_penalty = 1.5;     // cost multiplier on |steering|
    double steer_change_penalty = 1.5; // cost on change in steering
    double analytic_period = 5;     // attempt Dubins-to-goal every N expansions
    int max_iterations = 300000;
    double goal_xy_tol = 0.6;       // [m]
    double goal_yaw_tol = 0.20;     // [rad]
};

struct PlanResult {
    bool success = false;
    std::vector<Pose2D> path;       // dense, kinematically feasible
    double path_length = 0.0;       // [m]
    int nodes_expanded = 0;
    int iterations = 0;
    double plan_time_ms = 0.0;
};

class HybridAStar {
public:
    HybridAStar(const GridMap& map, const VehicleParams& veh, PlannerConfig cfg = {})
        : map_(map), veh_(veh), cfg_(cfg) {}

    PlanResult plan(const Pose2D& start, const Pose2D& goal);

private:
    const GridMap& map_;
    VehicleParams veh_;
    PlannerConfig cfg_;

    // Collision test for a single pose using the bounding-circle footprint.
    bool poseCollisionFree(const Pose2D& p) const;
    // Collision test along a densely sampled sequence of poses.
    bool pathCollisionFree(const std::vector<Pose2D>& poses) const;
    // Holonomic-with-obstacles heuristic field (Dijkstra from goal).
    std::vector<double> buildHolonomicHeuristic(const Pose2D& goal) const;
    double holonomicCost(const std::vector<double>& field, double x, double y) const;
};

}  // namespace ap
