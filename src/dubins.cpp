#include "ap/dubins.hpp"
#include <cmath>
#include <limits>

namespace ap {
namespace {

struct Word {
    bool valid = false;
    double t = 0, p = 0, q = 0;      // normalized: turns in rad, straight in units of r
    std::array<char, 3> types{};
    const char* name = "";
    double cost() const { return t + p + q; }
};

// The six Dubins words, in normalized coordinates. Formulas follow the
// classic closed-form solution (A. Walker's dubins library conventions).
Word LSL(double a, double b, double d) {
    double sa = std::sin(a), sb = std::sin(b), ca = std::cos(a), cb = std::cos(b);
    double c_ab = std::cos(a - b);
    Word w{}; w.types = {'L', 'S', 'L'}; w.name = "LSL";
    double p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb);
    if (p_sq < 0) return w;
    double tmp = std::atan2(cb - ca, d + sa - sb);
    w.t = mod2pi(-a + tmp);
    w.p = std::sqrt(p_sq);
    w.q = mod2pi(b - tmp);
    w.valid = true;
    return w;
}
Word RSR(double a, double b, double d) {
    double sa = std::sin(a), sb = std::sin(b), ca = std::cos(a), cb = std::cos(b);
    double c_ab = std::cos(a - b);
    Word w{}; w.types = {'R', 'S', 'R'}; w.name = "RSR";
    double p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa);
    if (p_sq < 0) return w;
    double tmp = std::atan2(ca - cb, d - sa + sb);
    w.t = mod2pi(a - tmp);
    w.p = std::sqrt(p_sq);
    w.q = mod2pi(-b + tmp);
    w.valid = true;
    return w;
}
Word LSR(double a, double b, double d) {
    double sa = std::sin(a), sb = std::sin(b), ca = std::cos(a), cb = std::cos(b);
    double c_ab = std::cos(a - b);
    Word w{}; w.types = {'L', 'S', 'R'}; w.name = "LSR";
    double p_sq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb);
    if (p_sq < 0) return w;
    double p = std::sqrt(p_sq);
    double tmp = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, p);
    w.t = mod2pi(-a + tmp);
    w.p = p;
    w.q = mod2pi(-b + tmp);
    w.valid = true;
    return w;
}
Word RSL(double a, double b, double d) {
    double sa = std::sin(a), sb = std::sin(b), ca = std::cos(a), cb = std::cos(b);
    double c_ab = std::cos(a - b);
    Word w{}; w.types = {'R', 'S', 'L'}; w.name = "RSL";
    double p_sq = d * d - 2 + 2 * c_ab - 2 * d * (sa + sb);
    if (p_sq < 0) return w;
    double p = std::sqrt(p_sq);
    double tmp = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, p);
    w.t = mod2pi(a - tmp);
    w.p = p;
    w.q = mod2pi(b - tmp);
    w.valid = true;
    return w;
}
Word RLR(double a, double b, double d) {
    double sa = std::sin(a), sb = std::sin(b), ca = std::cos(a), cb = std::cos(b);
    double c_ab = std::cos(a - b);
    Word w{}; w.types = {'R', 'L', 'R'}; w.name = "RLR";
    double tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0;
    if (std::fabs(tmp) > 1.0) return w;
    double p = mod2pi(2 * kPi - std::acos(tmp));
    double t = mod2pi(a - std::atan2(ca - cb, d - sa + sb) + p / 2.0);
    w.t = t;
    w.p = p;
    w.q = mod2pi(a - b - t + p);
    w.valid = true;
    return w;
}
Word LRL(double a, double b, double d) {
    double sa = std::sin(a), sb = std::sin(b), ca = std::cos(a), cb = std::cos(b);
    double c_ab = std::cos(a - b);
    Word w{}; w.types = {'L', 'R', 'L'}; w.name = "LRL";
    double tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (-sa + sb)) / 8.0;
    if (std::fabs(tmp) > 1.0) return w;
    double p = mod2pi(2 * kPi - std::acos(tmp));
    double t = mod2pi(-a - std::atan2(ca - cb, d + sa - sb) + p / 2.0);
    w.t = t;
    w.p = p;
    w.q = mod2pi(mod2pi(b) - a - t + p);
    w.valid = true;
    return w;
}

}  // namespace

DubinsPath dubinsShortest(const Pose2D& start, const Pose2D& goal, double r) {
    DubinsPath out;
    out.radius = r;
    out.start = start;
    if (r <= 0) return out;

    double dx = goal.x - start.x, dy = goal.y - start.y;
    double D = hypot2(dx, dy);
    double d = D / r;
    double theta = std::atan2(dy, dx);
    double a = mod2pi(start.theta - theta);
    double b = mod2pi(goal.theta - theta);

    Word words[6] = {LSL(a, b, d), RSR(a, b, d), LSR(a, b, d),
                     RSL(a, b, d), RLR(a, b, d), LRL(a, b, d)};

    const Word* best = nullptr;
    double best_cost = std::numeric_limits<double>::infinity();
    for (const Word& w : words) {
        if (w.valid && w.cost() < best_cost) {
            best_cost = w.cost();
            best = &w;
        }
    }
    if (!best) return out;

    out.valid = true;
    out.word = best->name;
    out.types = best->types;
    // Convert normalized lengths to real metres. Turn segments carry an angle,
    // straight segments carry a normalized distance; both scale by r.
    double norm[3] = {best->t, best->p, best->q};
    for (int i = 0; i < 3; ++i) out.len[i] = norm[i] * r;
    return out;
}

double dubinsLength(const Pose2D& start, const Pose2D& goal, double r) {
    DubinsPath p = dubinsShortest(start, goal, r);
    return p.valid ? p.length() : std::numeric_limits<double>::infinity();
}

std::vector<Pose2D> sampleDubins(const DubinsPath& path, double step) {
    std::vector<Pose2D> out;
    if (!path.valid) return out;
    Pose2D p = path.start;
    out.push_back(p);
    for (int seg = 0; seg < 3; ++seg) {
        double seg_len = path.len[seg];
        char type = path.types[seg];
        double kappa = (type == 'L') ? (1.0 / path.radius)
                       : (type == 'R') ? (-1.0 / path.radius)
                                       : 0.0;
        double traveled = 0.0;
        while (traveled < seg_len - 1e-9) {
            double ds = std::min(step, seg_len - traveled);
            // Exact integration of a constant-curvature arc.
            if (std::fabs(kappa) < 1e-9) {
                p.x += ds * std::cos(p.theta);
                p.y += ds * std::sin(p.theta);
            } else {
                double dtheta = kappa * ds;
                double nt = p.theta + dtheta;
                p.x += (std::sin(nt) - std::sin(p.theta)) / kappa;
                p.y += (-std::cos(nt) + std::cos(p.theta)) / kappa;
                p.theta = normalizeAngle(nt);
            }
            traveled += ds;
            out.push_back(p);
        }
    }
    return out;
}

}  // namespace ap
