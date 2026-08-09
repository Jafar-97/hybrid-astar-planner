// Demo driver: build a scenario, plan with Hybrid A*, track with pure pursuit,
// and dump logs (obstacles, planned path, executed trajectory, metrics) for the
// Python visualiser.
//
// Usage: planner_demo [scenario] [out_dir]
//   scenario: obstacles | scurve   (default: obstacles)
//   out_dir : directory for output files (default: ".")
#include "ap/controller.hpp"
#include "ap/hybrid_astar.hpp"
#include "ap/scenarios.hpp"
#include "ap/simulator.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

using namespace ap;

static void writeObstacles(const std::string& path, const GridMap& map) {
    std::ofstream f(path);
    f << "x0,y0,x1,y1\n";
    for (const auto& o : map.obstacles())
        f << o.x0 << ',' << o.y0 << ',' << o.x1 << ',' << o.y1 << '\n';
}

static void writePath(const std::string& path, const std::vector<Pose2D>& p) {
    std::ofstream f(path);
    f << "x,y,theta\n";
    for (const auto& s : p) f << s.x << ',' << s.y << ',' << s.theta << '\n';
}

static void writeTraj(const std::string& path, const std::vector<TrajSample>& t) {
    std::ofstream f(path);
    f << "t,x,y,theta,v,steer,target_x,target_y,cte\n";
    for (const auto& s : t)
        f << s.t << ',' << s.x << ',' << s.y << ',' << s.theta << ',' << s.v << ','
          << s.steer << ',' << s.target_x << ',' << s.target_y << ',' << s.cte
          << '\n';
}

int main(int argc, char** argv) {
    std::string scenario = (argc > 1) ? argv[1] : "obstacles";
    std::string out = (argc > 2) ? argv[2] : ".";

    Scenario sc = makeScenario(scenario);
    // Inflate obstacles by half the vehicle width plus a safety margin so the
    // planner can treat the footprint as a chain of point-discs.
    double inflation = 0.5 * sc.veh.width + 0.2;
    sc.map.inflate(inflation);

    PlannerConfig pcfg;
    HybridAStar planner(sc.map, sc.veh, pcfg);
    PlanResult plan = planner.plan(sc.start, sc.goal);

    std::cout << "=== Scenario: " << sc.name << " ===\n";
    std::cout << "Planning: "
              << (plan.success ? "SUCCESS" : "FAILURE") << "\n"
              << "  nodes expanded : " << plan.nodes_expanded << "\n"
              << "  iterations     : " << plan.iterations << "\n"
              << "  plan time      : " << plan.plan_time_ms << " ms\n"
              << "  path length    : " << plan.path_length << " m\n";

    writeObstacles(out + "/obstacles.csv", sc.map);
    writePath(out + "/path.csv", plan.path);

    TrackingMetrics tm;
    if (plan.success) {
        ControllerConfig ccfg;
        SimResult sim = simulate(plan.path, sc.start, sc.veh, ccfg);
        tm = sim.metrics;
        writeTraj(out + "/traj.csv", sim.log);
        std::cout << "Tracking:\n"
                  << "  completed      : " << (tm.completed ? "yes" : "no") << "\n"
                  << "  cross-track RMS: " << tm.cte_rms << " m\n"
                  << "  cross-track max: " << tm.cte_max << " m\n"
                  << "  avg speed      : " << tm.avg_speed << " m/s\n"
                  << "  sim time       : " << tm.sim_time << " s\n";
    }

    // Machine-readable summary.
    std::ofstream meta(out + "/meta.json");
    meta << "{\n"
         << "  \"scenario\": \"" << sc.name << "\",\n"
         << "  \"map\": {\"width\": " << sc.map.width() << ", \"height\": "
         << sc.map.height() << ", \"res\": " << sc.map.resolution() << "},\n"
         << "  \"vehicle\": {\"L\": " << sc.veh.wheelbase << ", \"width\": "
         << sc.veh.width << ", \"length\": " << sc.veh.length
         << ", \"min_turn_radius\": " << sc.veh.minTurnRadius() << "},\n"
         << "  \"start\": [" << sc.start.x << ", " << sc.start.y << ", "
         << sc.start.theta << "],\n"
         << "  \"goal\": [" << sc.goal.x << ", " << sc.goal.y << ", "
         << sc.goal.theta << "],\n"
         << "  \"inflation\": " << inflation << ",\n"
         << "  \"planning\": {\"success\": " << (plan.success ? "true" : "false")
         << ", \"nodes_expanded\": " << plan.nodes_expanded
         << ", \"iterations\": " << plan.iterations
         << ", \"plan_time_ms\": " << plan.plan_time_ms
         << ", \"path_length_m\": " << plan.path_length << "},\n"
         << "  \"tracking\": {\"completed\": " << (tm.completed ? "true" : "false")
         << ", \"cte_rms_m\": " << tm.cte_rms << ", \"cte_max_m\": " << tm.cte_max
         << ", \"avg_speed_mps\": " << tm.avg_speed << ", \"sim_time_s\": "
         << tm.sim_time << "}\n"
         << "}\n";

    return plan.success ? 0 : 1;
}
