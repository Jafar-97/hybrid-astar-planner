// Validates the Dubins solver by integrating each returned path and checking
// that (a) it lands exactly on the goal pose, and (b) its analytic length
// matches the arc length of the sampled curve and is >= the straight-line
// distance. This catches any sign/branch error in the closed-form equations.
#include "ap/dubins.hpp"
#include "test_util.hpp"
#include <vector>

using namespace ap;

int main() {
    double r = 5.0;
    struct Case { Pose2D a, b; };
    std::vector<Case> cases = {
        {{0, 0, 0}, {10, 0, 0}},
        {{0, 0, 0}, {0, 10, kPi / 2}},
        {{0, 0, 0}, {10, 10, kPi}},
        {{0, 0, 0}, {-8, 4, -kPi / 2}},
        {{2, 3, 1.0}, {20, -5, -2.0}},
        {{0, 0, 0}, {1, 0, kPi}},        // tight: forces a 3-turn word
        {{0, 0, kPi / 2}, {3, 1, -kPi / 2}},
    };

    int idx = 0;
    for (const auto& c : cases) {
        ++idx;
        DubinsPath p = dubinsShortest(c.a, c.b, r);
        test::check(p.valid, "case " + std::to_string(idx) + ": path valid");
        if (!p.valid) continue;

        auto samples = sampleDubins(p, 0.05);
        const Pose2D& end = samples.back();
        test::checkNear(end.x, c.b.x, 1e-2,
                        "case " + std::to_string(idx) + ": endpoint x");
        test::checkNear(end.y, c.b.y, 1e-2,
                        "case " + std::to_string(idx) + ": endpoint y");
        test::checkNear(normalizeAngle(end.theta - c.b.theta), 0.0, 1e-2,
                        "case " + std::to_string(idx) + ": endpoint heading");

        // Arc length of the sampled polyline ~ analytic length.
        double arc = 0.0;
        for (size_t i = 1; i < samples.size(); ++i)
            arc += dist(samples[i - 1], samples[i]);
        test::checkNear(arc, p.length(), 0.1,
                        "case " + std::to_string(idx) + ": length consistency");

        // Dubins length is at least the straight-line distance.
        test::check(p.length() >= dist(c.a, c.b) - 1e-6,
                    "case " + std::to_string(idx) + ": length >= straight line");
    }

    return test::summary("test_dubins");
}
