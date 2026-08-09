// Validates the Hybrid A* planner: it reaches the goal, the returned path is
// collision-free against the inflated map, and every consecutive pair of poses
// respects the vehicle's minimum turning radius (i.e. the path is feasible).
#include "ap/hybrid_astar.hpp"
#include "ap/scenarios.hpp"
#include "test_util.hpp"

using namespace ap;

static bool pathIsFree(const GridMap& map, const VehicleParams& veh,
                       const std::vector<Pose2D>& path) {
    const double halfL = 0.5 * veh.length;
    for (const auto& p : path)
        for (double s = -halfL; s <= halfL + 1e-6; s += 0.7) {
            double px = p.x + s * std::cos(p.theta);
            double py = p.y + s * std::sin(p.theta);
            if (!map.isFree(px, py)) return false;
        }
    return true;
}

// Max curvature between consecutive poses should not exceed 1/Rmin (+ slack).
static double maxCurvature(const std::vector<Pose2D>& path) {
    double kmax = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double ds = dist(path[i - 1], path[i]);
        if (ds < 1e-6) continue;
        double dth = std::fabs(normalizeAngle(path[i].theta - path[i - 1].theta));
        kmax = std::max(kmax, dth / ds);
    }
    return kmax;
}

int main() {
    // Empty arena (borders only) — a path must exist.
    {
        GridMap map(40.0, 40.0, 0.5);
        map.addObstacle({0, 0, 40, 1});
        map.addObstacle({0, 39, 40, 40});
        VehicleParams veh;
        map.inflate(0.5 * veh.width + 0.2);
        HybridAStar planner(map, veh);
        auto r = planner.plan({5, 20, 0}, {35, 20, 0});
        test::check(r.success, "empty arena: plan found");
        test::check(pathIsFree(map, veh, r.path), "empty arena: path collision-free");
        double kmax = maxCurvature(r.path);
        test::check(kmax <= 1.0 / veh.minTurnRadius() + 0.05,
                    "empty arena: curvature within limit");
    }

    // Built-in obstacle scenario — must route around obstacles.
    {
        Scenario sc = makeScenario("obstacles");
        sc.map.inflate(0.5 * sc.veh.width + 0.2);
        HybridAStar planner(sc.map, sc.veh);
        auto r = planner.plan(sc.start, sc.goal);
        test::check(r.success, "obstacles: plan found");
        test::check(pathIsFree(sc.map, sc.veh, r.path),
                    "obstacles: path collision-free");
        test::check(dist(r.path.back(), sc.goal) < 1.0,
                    "obstacles: ends at goal");
    }

    // S-curve scenario — a harder route.
    {
        Scenario sc = makeScenario("scurve");
        sc.map.inflate(0.5 * sc.veh.width + 0.2);
        HybridAStar planner(sc.map, sc.veh);
        auto r = planner.plan(sc.start, sc.goal);
        test::check(r.success, "scurve: plan found");
        test::check(pathIsFree(sc.map, sc.veh, r.path),
                    "scurve: path collision-free");
    }

    return test::summary("test_planner");
}
