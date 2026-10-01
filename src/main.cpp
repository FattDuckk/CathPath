#include "cathpath/centreline.hpp"
#include "cathpath/mesh_utils.hpp"
#include "cathpath/voxel_grid.hpp"
#include <algorithm>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: cathpath <mesh.ply> [voxel_mm] [out.csv]\n";
        return 1;
    }
    const double voxel = (argc >= 3) ? std::stod(argv[2]) : 1.0;
    const std::string outFile = (argc >= 4) ? argv[3] : "centreline.csv";

    // 1. load
    open3d::geometry::TriangleMesh mesh;
    if (!open3d::io::ReadTriangleMesh(argv[1], mesh)) {
        std::cerr << "failed to read " << argv[1] << "\n";
        return 1;
    }

    // 2. clean
    cleanMesh(mesh);
    printStats(mesh, "cleaned");
    std::cout << "mesh volume: " << meshVolume(mesh) << " mm^3\n";

    // 3. voxelise + fill
    Grid g = voxelise(mesh, voxel);
    floodFill(g);
    size_t inside = std::count(g.v.begin(), g.v.end(), INSIDE);
    size_t wall   = std::count(g.v.begin(), g.v.end(), WALL);
    std::cout << "grid " << g.nx << " x " << g.ny << " x " << g.nz
              << ", voxel volume: " << (inside + 0.5 * wall) * voxel * voxel * voxel << " mm^3\n";

    // 4. distance from wall
    auto dist = distanceToWall(g);
    float maxD = *std::max_element(dist.begin(), dist.end());
    std::cout << "max distance from wall: " << maxD * voxel << " mm\n";

    // 5. endpoints: farthest-apart voxels, then nudged onto the axis
    int seed = std::find(g.v.begin(), g.v.end(), INSIDE) - g.v.begin();
    int endA = farthestVoxel(g, seed);
    int endB = farthestVoxel(g, endA);
    endA = centredEnd(g, dist, endA, (int)maxD);
    endB = centredEnd(g, dist, endB, (int)maxD);

    // 6. centreline
    auto pts = toWorld(g, centrePath(g, dist, endA, endB));
    size_t core = pts.size();
    extendToWall(g, pts);
    std::cout << "centreline: " << pts.size() << " points (" << pts.size() - core
              << " added by extension), " << pathLength(pts) << " mm\n";

    saveCsv(pts, outFile);
    std::cout << "saved " << outFile << "\n";
}