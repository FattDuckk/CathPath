#include "cathpath/mesh_utils.hpp"
#include <iostream>

using open3d::geometry::TriangleMesh;

void cleanMesh(TriangleMesh& m) {
    m.RemoveDuplicatedVertices();
    m.MergeCloseVertices(1e-3);
    m.RemoveDuplicatedTriangles();
    m.RemoveDegenerateTriangles();
}

void printStats(const TriangleMesh& m, const std::string& label) {
    auto [ids, counts, areas] = m.ClusterConnectedTriangles();
    std::cout << "--- " << label << " ---\n"
              << m.vertices_.size() << " vertices, " << m.triangles_.size() << " triangles\n"
              << "edge manifold: " << std::boolalpha << m.IsEdgeManifold(false) << "\n"
              << m.GetNonManifoldEdges(false).size() << " boundary edges\n"
              << counts.size() << " connected pieces\n";
}

double meshVolume(const TriangleMesh& m) {
    double vol = 0;
    for (const auto& t : m.triangles_) {
        const auto& a = m.vertices_[t(0)];
        const auto& b = m.vertices_[t(1)];
        const auto& c = m.vertices_[t(2)];
        vol += a.dot(b.cross(c));
    }
    return std::abs(vol) / 6.0;
}