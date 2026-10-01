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
    bool inBounds(int x, int y, int z) const {
        return x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz;
    }
};

Grid voxelise(const open3d::geometry::TriangleMesh& m, double voxel);
void floodFill(Grid& g);   // YOUR TASK
std::shared_ptr<open3d::geometry::VoxelGrid> toVoxelGrid(const Grid& g, uint8_t label);