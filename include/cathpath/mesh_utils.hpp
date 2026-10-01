#pragma once
#include <open3d/Open3D.h>
#include <string>

void cleanMesh(open3d::geometry::TriangleMesh& m);
void printStats(const open3d::geometry::TriangleMesh& m, const std::string& label);
double meshVolume(const open3d::geometry::TriangleMesh& m);