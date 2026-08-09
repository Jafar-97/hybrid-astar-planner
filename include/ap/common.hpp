#pragma once
// Common geometry and math utilities for the autonomy planner.
#include <cmath>
#include <vector>

namespace ap {

constexpr double kPi = 3.14159265358979323846;

// A 2D pose: position (x, y) in metres and heading theta in radians.
struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// Wrap an angle to (-pi, pi].
inline double normalizeAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

// Wrap an angle to [0, 2pi).
inline double mod2pi(double a) {
    double r = std::fmod(a, 2.0 * kPi);
    if (r < 0.0) r += 2.0 * kPi;
    return r;
}

inline double hypot2(double dx, double dy) { return std::sqrt(dx * dx + dy * dy); }

// Euclidean distance between two poses (ignores heading).
inline double dist(const Pose2D& a, const Pose2D& b) {
    return hypot2(a.x - b.x, a.y - b.y);
}

}  // namespace ap
