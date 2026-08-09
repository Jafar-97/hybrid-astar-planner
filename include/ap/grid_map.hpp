#pragma once
// 2D occupancy grid / costmap. Obstacles are rasterised into cells and then
// "inflated" by the vehicle's collision radius, so the planner can treat the
// vehicle as a point and still keep a safety margin around obstacles.
#include "ap/common.hpp"
#include <cstdint>
#include <vector>

namespace ap {

struct Obstacle {  // axis-aligned rectangle in world coordinates
    double x0, y0, x1, y1;
};

class GridMap {
public:
    GridMap(double width_m, double height_m, double resolution);

    void addObstacle(const Obstacle& o);
    // Grow occupied regions by `radius` metres (Chebyshev/Euclidean inflation).
    void inflate(double radius);

    bool inBounds(double x, double y) const;
    // True if the world point (x, y) lies in a free (unoccupied) cell.
    bool isFree(double x, double y) const;

    int cols() const { return cols_; }
    int rows() const { return rows_; }
    double resolution() const { return res_; }
    double width() const { return width_; }
    double height() const { return height_; }

    // Cell <-> world helpers.
    int cellIndex(int cx, int cy) const { return cy * cols_ + cx; }
    bool worldToCell(double x, double y, int& cx, int& cy) const;
    bool occupied(int cx, int cy) const;

    const std::vector<uint8_t>& data() const { return occ_; }
    const std::vector<Obstacle>& obstacles() const { return raw_obstacles_; }

private:
    double width_, height_, res_;
    int cols_, rows_;
    std::vector<uint8_t> occ_;  // 0 = free, 1 = occupied
    std::vector<Obstacle> raw_obstacles_;
};

}  // namespace ap
