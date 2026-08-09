#pragma once
#include <cmath>
#include <cstdio>
#include <string>

// Minimal dependency-free test harness so the project builds without gtest.
namespace test {
inline int g_failures = 0;

inline void check(bool cond, const std::string& msg) {
    if (!cond) {
        std::printf("  [FAIL] %s\n", msg.c_str());
        ++g_failures;
    } else {
        std::printf("  [ok]   %s\n", msg.c_str());
    }
}

inline void checkNear(double a, double b, double tol, const std::string& msg) {
    bool ok = std::fabs(a - b) <= tol;
    if (!ok)
        std::printf("  [FAIL] %s (|%.6f - %.6f| = %.6f > %.6f)\n", msg.c_str(), a,
                    b, std::fabs(a - b), tol);
    else
        std::printf("  [ok]   %s\n", msg.c_str());
    if (!ok) ++g_failures;
}

inline int summary(const char* name) {
    if (g_failures == 0) {
        std::printf("[PASS] %s: all checks passed\n", name);
        return 0;
    }
    std::printf("[FAIL] %s: %d check(s) failed\n", name, g_failures);
    return 1;
}
}  // namespace test
