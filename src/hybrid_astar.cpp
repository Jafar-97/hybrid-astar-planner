#include "ap/hybrid_astar.hpp"
#include "ap/dubins.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>

namespace ap {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Node {
    Pose2D pose;
    double g = 0.0;
    double f = 0.0;
    double steer = 0.0;
    int parent = -1;
};
}  // namespace

bool HybridAStar::poseCollisionFree(const Pose2D& p) const {
    if (!map_.inBounds(p.x, p.y)) return false;
    // Model the footprint as a chain of discs along the centreline; the map is
    // pre-inflated by the disc radius, so each disc reduces to a point test.
    const double halfL = 0.5 * veh_.length;
    for (double s = -halfL; s <= halfL + 1e-6; s += 0.7) {
        double px = p.x + s * std::cos(p.theta);
        double py = p.y + s * std::sin(p.theta);
        if (!map_.isFree(px, py)) return false;
    }
    return true;
}

bool HybridAStar::pathCollisionFree(const std::vector<Pose2D>& poses) const {
    for (const Pose2D& p : poses)
        if (!poseCollisionFree(p)) return false;
    return true;
}

std::vector<double> HybridAStar::buildHolonomicHeuristic(const Pose2D& goal) const {
    const int cols = map_.cols(), rows = map_.rows();
    std::vector<double> field(static_cast<size_t>(cols) * rows, kInf);
    int gx, gy;
    if (!map_.worldToCell(goal.x, goal.y, gx, gy)) return field;

    using QI = std::pair<double, int>;
    std::priority_queue<QI, std::vector<QI>, std::greater<QI>> pq;
    int gi = map_.cellIndex(gx, gy);
    field[gi] = 0.0;
    pq.push({0.0, gi});
    const double res = map_.resolution();
    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!pq.empty()) {
        auto [c, idx] = pq.top();
        pq.pop();
        if (c > field[idx]) continue;
        int cx = idx % cols, cy = idx / cols;
        for (int k = 0; k < 8; ++k) {
            int nx = cx + dx[k], ny = cy + dy[k];
            if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
            if (map_.occupied(nx, ny)) continue;
            double step = (k < 4 ? 1.0 : 1.41421356) * res;
            int nidx = map_.cellIndex(nx, ny);
            double nc = c + step;
            if (nc < field[nidx]) {
                field[nidx] = nc;
                pq.push({nc, nidx});
            }
        }
    }
    return field;
}

double HybridAStar::holonomicCost(const std::vector<double>& field, double x,
                                  double y) const {
    int cx, cy;
    if (!map_.worldToCell(x, y, cx, cy)) return kInf;
    double v = field[map_.cellIndex(cx, cy)];
    return v;  // may be INF if unreachable
}

PlanResult HybridAStar::plan(const Pose2D& start, const Pose2D& goal) {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    PlanResult result;

    const double R = veh_.minTurnRadius();
    const double res = cfg_.xy_resolution;
    const double dtheta_bin = 2.0 * kPi / cfg_.theta_bins;

    auto keyOf = [&](const Pose2D& p) -> int64_t {
        int xi = static_cast<int>(std::floor(p.x / res));
        int yi = static_cast<int>(std::floor(p.y / res));
        int ti = static_cast<int>(std::floor(mod2pi(p.theta) / dtheta_bin)) %
                 cfg_.theta_bins;
        // Pack into a single 64-bit key (offsets keep values non-negative-ish).
        int64_t k = (static_cast<int64_t>(xi + 100000) << 34) ^
                    (static_cast<int64_t>(yi + 100000) << 8) ^
                    static_cast<int64_t>(ti);
        return k;
    };

    std::vector<double> hfield = buildHolonomicHeuristic(goal);
    auto heuristic = [&](const Pose2D& p) -> double {
        double h_nh = dubinsLength(p, goal, R);
        double h_h = holonomicCost(hfield, p.x, p.y);
        double h = std::max(h_nh, h_h);
        return std::isfinite(h) ? h : h_nh;  // fall back if holo unreachable
    };

    // Steering samples in [-max_steer, max_steer].
    std::vector<double> steers;
    if (cfg_.n_steer <= 1) {
        steers = {0.0};
    } else {
        for (int i = 0; i < cfg_.n_steer; ++i) {
            double s = -veh_.max_steer +
                       2.0 * veh_.max_steer * i / (cfg_.n_steer - 1);
            steers.push_back(s);
        }
    }

    std::vector<Node> nodes;
    nodes.reserve(1 << 16);
    std::unordered_map<int64_t, double> best_g;  // closed/open best cost per cell

    Node s0;
    s0.pose = start;
    s0.g = 0.0;
    s0.f = heuristic(start);
    s0.parent = -1;
    nodes.push_back(s0);
    best_g[keyOf(start)] = 0.0;

    using QI = std::pair<double, int>;
    std::priority_queue<QI, std::vector<QI>, std::greater<QI>> open;
    open.push({s0.f, 0});

    const int substeps = 10;
    const double ds = cfg_.step_size / substeps;
    int goal_node = -1;
    std::vector<Pose2D> analytic_tail;
    int iter = 0;

    auto tryAnalytic = [&](int idx) -> bool {
        DubinsPath dp = dubinsShortest(nodes[idx].pose, goal, R);
        if (!dp.valid) return false;
        std::vector<Pose2D> samp = sampleDubins(dp, 0.3);
        if (!pathCollisionFree(samp)) return false;
        analytic_tail = samp;
        goal_node = idx;
        return true;
    };

    while (!open.empty() && iter < cfg_.max_iterations) {
        auto [f_cur, idx] = open.top();
        open.pop();
        ++iter;
        Node cur = nodes[idx];
        if (f_cur > cur.f + 1e-9) continue;  // stale entry

        // Goal reached by discretisation?
        if (dist(cur.pose, goal) < cfg_.goal_xy_tol &&
            std::fabs(normalizeAngle(cur.pose.theta - goal.theta)) <
                cfg_.goal_yaw_tol) {
            goal_node = idx;
            break;
        }
        // Periodic analytic expansion (cheaper when we are close).
        bool near = dist(cur.pose, goal) < 15.0;
        if (near && (iter % static_cast<int>(cfg_.analytic_period) == 0)) {
            if (tryAnalytic(idx)) break;
        }

        result.nodes_expanded++;
        for (double steer : steers) {
            Pose2D p = cur.pose;
            bool ok = true;
            for (int k = 0; k < substeps; ++k) {
                p = bicycleStep(p, steer, ds, veh_.wheelbase);
                if (!poseCollisionFree(p)) { ok = false; break; }
            }
            if (!ok) continue;

            double move_cost =
                cfg_.step_size * (1.0 + cfg_.steer_penalty * std::fabs(steer) +
                                  cfg_.steer_change_penalty *
                                      std::fabs(steer - cur.steer));
            double g_new = cur.g + move_cost;
            int64_t key = keyOf(p);
            auto it = best_g.find(key);
            if (it != best_g.end() && it->second <= g_new + 1e-6) continue;
            best_g[key] = g_new;

            Node nn;
            nn.pose = p;
            nn.g = g_new;
            nn.f = g_new + heuristic(p);
            nn.steer = steer;
            nn.parent = idx;
            int nidx = static_cast<int>(nodes.size());
            nodes.push_back(nn);
            open.push({nn.f, nidx});
        }
    }

    result.iterations = iter;
    if (goal_node < 0) {
        auto t1 = clock::now();
        result.plan_time_ms =
            std::chrono::duration<double, std::milli>(t1 - t0).count();
        return result;
    }

    // Reconstruct the sparse node chain, then re-integrate each edge densely so
    // the returned path is smooth and exactly follows the bicycle model.
    std::vector<int> chain;
    for (int i = goal_node; i >= 0; i = nodes[i].parent) chain.push_back(i);
    std::reverse(chain.begin(), chain.end());

    std::vector<Pose2D> dense;
    dense.push_back(nodes[chain.front()].pose);
    for (size_t c = 1; c < chain.size(); ++c) {
        Pose2D p = nodes[chain[c - 1]].pose;
        double steer = nodes[chain[c]].steer;
        for (int k = 0; k < substeps; ++k) {
            p = bicycleStep(p, steer, ds, veh_.wheelbase);
            dense.push_back(p);
        }
    }
    for (size_t i = 1; i < analytic_tail.size(); ++i)
        dense.push_back(analytic_tail[i]);

    double len = 0.0;
    for (size_t i = 1; i < dense.size(); ++i) len += dist(dense[i - 1], dense[i]);

    result.success = true;
    result.path = std::move(dense);
    result.path_length = len;

    auto t1 = clock::now();
    result.plan_time_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

}  // namespace ap
