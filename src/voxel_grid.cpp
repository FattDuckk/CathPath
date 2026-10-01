#include "cathpath/voxel_grid.hpp"
#include <iostream>
#include <queue>

using open3d::geometry::TriangleMesh;
using open3d::geometry::VoxelGrid;

Grid voxelise(const TriangleMesh& m, double voxel) {
    auto bb = m.GetAxisAlignedBoundingBox();
    Eigen::Vector3d pad = Eigen::Vector3d::Constant(2 * voxel);
    Eigen::Vector3d minB = bb.min_bound_ - pad, maxB = bb.max_bound_ + pad;

    auto vg = VoxelGrid::CreateFromTriangleMeshWithinBounds(m, voxel, minB, maxB);

    Grid g;
    g.voxel = voxel;
    g.origin = minB;
    Eigen::Vector3i dims = ((maxB - minB) / voxel).array().ceil().cast<int>();
    g.nx = dims.x(); g.ny = dims.y(); g.nz = dims.z();
    g.v.assign(g.nx * g.ny * g.nz, EMPTY);

    for (const auto& vx : vg->GetVoxels()) {
        const auto& i = vx.grid_index_;
        if (g.inBounds(i.x(), i.y(), i.z())) g.v[g.idx(i.x(), i.y(), i.z())] = WALL;
    }
    return g;
}

void floodFill(Grid& g) {
    // 6 face-neighbours: ±x, ±y, ±z
    const int dirs[6][3] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};

    std::queue<Eigen::Vector3i> q;
    g.v[g.idx(0, 0, 0)] = OUTSIDE;     // corner is guaranteed outside thanks to the padding
    q.emplace(0, 0, 0);

    while (!q.empty()) {
        Eigen::Vector3i p = q.front();
        q.pop();
        for (const auto& d : dirs) {
            int x = p.x() + d[0], y = p.y() + d[1], z = p.z() + d[2];
            if (!g.inBounds(x, y, z)) continue;
            uint8_t& cell = g.v[g.idx(x, y, z)];
            if (cell != EMPTY) continue;   // wall or already visited
            cell = OUTSIDE;
            q.emplace(x, y, z);
        }
    }

    // anything the fill couldn't reach is enclosed by the wall
    size_t inside = 0, outside = 0, wall = 0;
    for (auto& c : g.v) {
        if (c == EMPTY) c = INSIDE;
        if (c == INSIDE) ++inside;
        else if (c == OUTSIDE) ++outside;
        else ++wall;
    }
    std::cout << "outside " << outside << ", wall " << wall << ", inside " << inside << "\n";
}

// turn every voxel with a given label into an Open3D VoxelGrid for drawing
std::shared_ptr<VoxelGrid> toDistanceVoxelGrid(const Grid& g, const std::vector<float>& d, float minFrac) {
    float maxD = *std::max_element(d.begin(), d.end());
    auto out = std::make_shared<VoxelGrid>();
    out->voxel_size_ = g.voxel;
    out->origin_ = g.origin;
    for (int i = 0; i < (int)g.v.size(); ++i) {
        if (g.v[i] != INSIDE) continue;
        double t = d[i] / maxD;
        if (t < minFrac) continue;
        out->AddVoxel(open3d::geometry::Voxel(g.xyz(i), Eigen::Vector3d(t, 0.2, 1.0 - t)));
    }
    return out;
}