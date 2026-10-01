#pragma once
#include "cathpath/voxel_grid.hpp"
#include <string>
#include <vector>

// distance (in voxels) from each INSIDE voxel to the nearest WALL voxel, -1 elsewhere
std::vector<float> distanceToWall(const Grid& g);

// BFS through INSIDE voxels from start; returns the farthest voxel reached
int farthestVoxel(const Grid& g, int start);

// most central voxel within `radius` steps of `start` (moves an endpoint off the wall onto the axis)
int centredEnd(const Grid& g, const std::vector<float>& dist, int start, int radius);

// Dijkstra from src to dst, preferring voxels far from the wall
std::vector<int> centrePath(const Grid& g, const std::vector<float>& dist, int src, int dst);

// voxel indices -> world coordinates (mm), at voxel centres
std::vector<Eigen::Vector3d> toWorld(const Grid& g, const std::vector<int>& path);

// extend both ends in a straight line along their final direction until they reach the wall
void extendToWall(const Grid& g, std::vector<Eigen::Vector3d>& pts, int lookback = 8);

double pathLength(const std::vector<Eigen::Vector3d>& pts);
void saveCsv(const std::vector<Eigen::Vector3d>& pts, const std::string& file);