#include "ap/scenarios.hpp"

namespace ap {

std::vector<std::string> scenarioNames() { return {"obstacles", "scurve"}; }

Scenario makeScenario(const std::string& name) {
    VehicleParams veh;  // default car

    if (name == "scurve") {
        // Two staggered walls forming an S: the car must swing up, then down.
        GridMap map(60.0, 40.0, 0.5);
        map.addObstacle({0.0, 0.0, 60.0, 1.0});     // bottom border
        map.addObstacle({0.0, 39.0, 60.0, 40.0});   // top border
        map.addObstacle({22.0, 0.0, 26.0, 26.0});   // wall from bottom, gap on top
        map.addObstacle({36.0, 14.0, 40.0, 40.0});  // wall from top, gap on bottom
        Pose2D start{5.0, 20.0, 0.0};
        Pose2D goal{55.0, 20.0, 0.0};
        return Scenario{"scurve", std::move(map), start, goal, veh};
    }

    // Default: a field of scattered rectangular obstacles.
    GridMap map(60.0, 40.0, 0.5);
    map.addObstacle({0.0, 0.0, 60.0, 1.0});
    map.addObstacle({0.0, 39.0, 60.0, 40.0});
    map.addObstacle({15.0, 8.0, 20.0, 24.0});
    map.addObstacle({28.0, 18.0, 33.0, 40.0});
    map.addObstacle({30.0, 0.0, 34.0, 10.0});
    map.addObstacle({42.0, 10.0, 47.0, 30.0});
    Pose2D start{5.0, 20.0, 0.0};
    Pose2D goal{56.0, 12.0, 0.0};
    return Scenario{"obstacles", std::move(map), start, goal, veh};
}

}  // namespace ap
