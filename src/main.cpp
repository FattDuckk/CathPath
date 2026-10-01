#include "cathpath/mesh_utils.hpp"
#include "cathpath/voxel_grid.hpp"
#include <algorithm>
#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "usage: cathpath <mesh.ply> [voxel_mm]\n"; return 1; }
    double voxel = (argc >= 3) ? std::stod(argv[2]) : 1.0;

    // Force X11 so the Open3D viewer works on Wayland
    setenv("XDG_SESSION_TYPE", "x11", 1);
    unsetenv("WAYLAND_DISPLAY");

    // 1. load
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    if (!open3d::io::ReadTriangleMesh(argv[1], *mesh)) {
        std::cerr << "failed to read " << argv[1] << "\n"; return 1;
    }

    // 2. clean
    cleanMesh(*mesh);
    printStats(*mesh, "cleaned");
    std::cout << "volume: " << meshVolume(*mesh) << " mm^3\n";

    // 3. voxelise + fill
    Grid g = voxelise(*mesh, voxel);
    std::cout << "grid " << g.nx << " x " << g.ny << " x " << g.nz << "\n";
    floodFill(g);

    size_t inside = std::count(g.v.begin(), g.v.end(), INSIDE);
    size_t wall   = std::count(g.v.begin(), g.v.end(), WALL);
    double est = (inside + 0.5 * wall) * voxel * voxel * voxel;
    std::cout << "voxel volume estimate: " << est << " mm^3\n";

    open3d::visualization::DrawGeometries({toVoxelGrid(g, INSIDE)}, "inside voxels");
}