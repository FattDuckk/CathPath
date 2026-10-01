#pragma once
#include <open3d/Open3D.h>
#include <cstdint>
#include <vector>

enum : uint8_t { EMPTY = 0, WALL = 1, OUTSIDE = 2, INSIDE = 3 };

struct Grid {
    int nx = 0, ny = 0, nz = 0;
    double voxel = 1.0;
    Eigen::Vector3d origin;
    std::vector<uint8_t> v;

    int idx(int x, int y, int z) const { return x + nx * (y + ny * z); }
    Eigen::Vector3i xyz(int i) const { return {i % nx, (i / nx) % ny, i / (nx * ny)}; }
    bool inBounds(int x, int y, int z) const {
        return x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz;
    }
        // voxel index containing world point p (mm), or -1 if outside the grid
    int idxAt(const Eigen::Vector3d& p) const {
        Eigen::Vector3i c = ((p - origin) / voxel).array().floor().cast<int>();
        return inBounds(c.x(), c.y(), c.z()) ? idx(c.x(), c.y(), c.z()) : -1;
    }
};

Grid voxelise(const open3d::geometry::TriangleMesh& m, double voxel);
void floodFill(Grid& g);
std::shared_ptr<open3d::geometry::VoxelGrid> toVoxelGrid(const Grid& g, uint8_t label);
std::shared_ptr<open3d::geometry::VoxelGrid> toDistanceVoxelGrid(
    const Grid& g, const std::vector<float>& d, float minFrac = 0.0f);