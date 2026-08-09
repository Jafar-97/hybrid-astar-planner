#pragma once
// Dubins shortest-path curves: the shortest forward-only path between two
// oriented poses for a car with a fixed minimum turning radius. Used in two
// places by the Hybrid A* planner:
//   1) as an admissible "non-holonomic-without-obstacles" heuristic, and
//   2) as an analytic expansion that snaps the search directly to the goal
//      when the straight-ish shot is collision-free.
#include "ap/common.hpp"
#include <array>
#include <string>
#include <vector>

namespace ap {

// One of the six Dubins words (e.g. LSL, RSR, LSR, RSL, RLR, LRL).
struct DubinsPath {
    bool valid = false;
    std::string word;            // segment types, e.g. "LSL"
    std::array<char, 3> types{}; // 'L','S','R'
    std::array<double, 3> len{}; // segment lengths in *real metres*
    double radius = 1.0;
    Pose2D start;

    double length() const { return len[0] + len[1] + len[2]; }
};

// Compute the shortest Dubins path from `start` to `goal` with turning radius r.
DubinsPath dubinsShortest(const Pose2D& start, const Pose2D& goal, double r);

// Just the length (INFINITY if none) — convenient for the heuristic.
double dubinsLength(const Pose2D& start, const Pose2D& goal, double r);

// Sample poses along a path at the given arc-length step (metres).
std::vector<Pose2D> sampleDubins(const DubinsPath& path, double step);

}  // namespace ap
