#include "ap/grid_map.hpp"
#include <algorithm>
#include <cmath>

namespace ap {

GridMap::GridMap(double width_m, double height_m, double resolution)
    : width_(width_m), height_(height_m), res_(resolution) {
    cols_ = static_cast<int>(std::ceil(width_m / resolution));
    rows_ = static_cast<int>(std::ceil(height_m / resolution));
    occ_.assign(static_cast<size_t>(cols_) * rows_, 0);
}

bool GridMap::worldToCell(double x, double y, int& cx, int& cy) const {
    cx = static_cast<int>(std::floor(x / res_));
    cy = static_cast<int>(std::floor(y / res_));
    return cx >= 0 && cx < cols_ && cy >= 0 && cy < rows_;
}

bool GridMap::inBounds(double x, double y) const {
    return x >= 0.0 && y >= 0.0 && x < width_ && y < height_;
}

bool GridMap::occupied(int cx, int cy) const {
    if (cx < 0 || cx >= cols_ || cy < 0 || cy >= rows_) return true;  // outside = blocked
    return occ_[cellIndex(cx, cy)] != 0;
}

bool GridMap::isFree(double x, double y) const {
    int cx, cy;
    if (!worldToCell(x, y, cx, cy)) return false;
    return occ_[cellIndex(cx, cy)] == 0;
}

void GridMap::addObstacle(const Obstacle& o) {
    raw_obstacles_.push_back(o);
    int cx0, cy0, cx1, cy1;
    worldToCell(std::min(o.x0, o.x1), std::min(o.y0, o.y1), cx0, cy0);
    worldToCell(std::max(o.x0, o.x1), std::max(o.y0, o.y1), cx1, cy1);
    cx0 = std::clamp(cx0, 0, cols_ - 1);
    cy0 = std::clamp(cy0, 0, rows_ - 1);
    cx1 = std::clamp(cx1, 0, cols_ - 1);
    cy1 = std::clamp(cy1, 0, rows_ - 1);
    for (int cy = cy0; cy <= cy1; ++cy)
        for (int cx = cx0; cx <= cx1; ++cx) occ_[cellIndex(cx, cy)] = 1;
}

void GridMap::inflate(double radius) {
    int r = static_cast<int>(std::ceil(radius / res_));
    if (r <= 0) return;
    std::vector<uint8_t> src = occ_;
    for (int cy = 0; cy < rows_; ++cy) {
        for (int cx = 0; cx < cols_; ++cx) {
            if (src[cellIndex(cx, cy)] == 0) continue;
            for (int dy = -r; dy <= r; ++dy) {
                for (int dx = -r; dx <= r; ++dx) {
                    if (dx * dx + dy * dy > r * r) continue;  // circular kernel
                    int nx = cx + dx, ny = cy + dy;
                    if (nx < 0 || nx >= cols_ || ny < 0 || ny >= rows_) continue;
                    occ_[cellIndex(nx, ny)] = 1;
                }
            }
        }
    }
}

}  // namespace ap
