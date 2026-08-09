#pragma once
// Built-in benchmark scenarios. Each returns a costmap, start/goal poses and a
// vehicle so the demo is fully reproducible without external map files.
#include "ap/grid_map.hpp"
#include "ap/vehicle.hpp"
#include <string>
#include <vector>

namespace ap {

struct Scenario {
    std::string name;
    GridMap map;
    Pose2D start;
    Pose2D goal;
    VehicleParams veh;
};

// name is one of: "obstacles", "scurve". Returns "obstacles" for unknown names.
Scenario makeScenario(const std::string& name);
std::vector<std::string> scenarioNames();

}  // namespace ap
