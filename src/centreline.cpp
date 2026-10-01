#include "cathpath/centreline.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>

namespace {
    // all 26 neighbours: every combination of -1/0/+1 except (0,0,0)
    std::vector<Eigen::Vector3i> neighbours26() {
        std::vector<Eigen::Vector3i> n;
        for (int dz = -1; dz <= 1; ++dz)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                    if (dx || dy || dz) n.emplace_back(dx, dy, dz);
        return n;
    }

    // extend the back end of pts along its final direction while still inside the vessel
    void extendBack(const Grid& g, std::vector<Eigen::Vector3d>& pts, int lookback) {
        int n = (int)pts.size();
        if (n < 2) return;
        int k = std::min(lookback, n - 1);
        Eigen::Vector3d dir = (pts[n - 1] - pts[n - 1 - k]).normalized();   // averaged over k steps, not just the last zigzag

        Eigen::Vector3d p = pts.back();
        for (int s = 0; s < 1000; ++s) {        // safety cap
            p += dir * g.voxel;
            int i = g.idxAt(p);
            if (i < 0 || g.v[i] != INSIDE) break;   // hit the wall
            pts.push_back(p);
        }
    }
}

std::vector<float> distanceToWall(const Grid& g) {
    const int dirs[6][3] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};
    std::vector<float> d(g.v.size(), -1.0f);   // -1 = not reached
    std::queue<int> q;

    // every wall voxel is a start point at distance 0
    for (int i = 0; i < (int)g.v.size(); ++i)
        if (g.v[i] == WALL) { d[i] = 0; q.push(i); }

    // spread inwards one layer at a time
    while (!q.empty()) {
        int i = q.front(); q.pop();
        Eigen::Vector3i p = g.xyz(i);
        for (const auto& dir : dirs) {
            int x = p.x() + dir[0], y = p.y() + dir[1], z = p.z() + dir[2];
            if (!g.inBounds(x, y, z)) continue;
            int j = g.idx(x, y, z);
            if (g.v[j] != INSIDE || d[j] >= 0) continue;
            d[j] = d[i] + 1;
            q.push(j);
        }
    }
    return d;
}

int farthestVoxel(const Grid& g, int start) {
    static const auto nbrs = neighbours26();
    std::vector<bool> seen(g.v.size(), false);
    std::queue<int> q;
    q.push(start);
    seen[start] = true;
    int last = start;

    while (!q.empty()) {
        int i = q.front(); q.pop();
        last = i;                               // final voxel popped is the farthest
        Eigen::Vector3i p = g.xyz(i);
        for (const auto& o : nbrs) {
            Eigen::Vector3i n = p + o;
            if (!g.inBounds(n.x(), n.y(), n.z())) continue;
            int j = g.idx(n.x(), n.y(), n.z());
            if (g.v[j] != INSIDE || seen[j]) continue;
            seen[j] = true;
            q.push(j);
        }
    }
    return last;
}

int centredEnd(const Grid& g, const std::vector<float>& dist, int start, int radius) {
    static const auto nbrs = neighbours26();
    std::vector<int> depth(g.v.size(), -1);
    std::queue<int> q;
    q.push(start);
    depth[start] = 0;
    int best = start;

    while (!q.empty()) {
        int i = q.front(); q.pop();
        if (dist[i] > dist[best]) best = i;     // strict > keeps the closest on ties
        if (depth[i] == radius) continue;
        Eigen::Vector3i p = g.xyz(i);
        for (const auto& o : nbrs) {
            Eigen::Vector3i n = p + o;
            if (!g.inBounds(n.x(), n.y(), n.z())) continue;
            int j = g.idx(n.x(), n.y(), n.z());
            if (g.v[j] != INSIDE || depth[j] >= 0) continue;
            depth[j] = depth[i] + 1;
            q.push(j);
        }
    }
    return best;
}

std::vector<int> centrePath(const Grid& g, const std::vector<float>& dist, int src, int dst) {
    static const auto nbrs = neighbours26();
    const float INF = std::numeric_limits<float>::infinity();
    std::vector<float> cost(g.v.size(), INF);
    std::vector<int> parent(g.v.size(), -1);

    using Item = std::pair<float, int>;   // (cost so far, voxel)
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
    cost[src] = 0;
    pq.emplace(0.0f, src);

    while (!pq.empty()) {
        auto [c, i] = pq.top(); pq.pop();
        if (c > cost[i]) continue;              // stale entry
        if (i == dst) break;
        Eigen::Vector3i p = g.xyz(i);
        for (const auto& o : nbrs) {
            Eigen::Vector3i n = p + o;
            if (!g.inBounds(n.x(), n.y(), n.z())) continue;
            int j = g.idx(n.x(), n.y(), n.z());
            if (g.v[j] != INSIDE) continue;
            float step = o.cast<float>().norm();        // 1, 1.41 or 1.73
            float w = step / (dist[j] * dist[j]);       // cheap in the middle, pricey near the wall
            if (c + w < cost[j]) {
                cost[j] = c + w;
                parent[j] = i;
                pq.emplace(cost[j], j);
            }
        }
    }

    std::vector<int> path;
    for (int i = dst; i != -1; i = parent[i]) path.push_back(i);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Eigen::Vector3d> toWorld(const Grid& g, const std::vector<int>& path) {
    std::vector<Eigen::Vector3d> pts;
    for (int i : path)
        pts.push_back(g.origin + (g.xyz(i).cast<double>() + Eigen::Vector3d::Constant(0.5)) * g.voxel);
    return pts;
}

void extendToWall(const Grid& g, std::vector<Eigen::Vector3d>& pts, int lookback) {
    extendBack(g, pts, lookback);               // extend the end
    std::reverse(pts.begin(), pts.end());
    extendBack(g, pts, lookback);               // extend the start (now at the back)
    std::reverse(pts.begin(), pts.end());       // restore original order
}

double pathLength(const std::vector<Eigen::Vector3d>& pts) {
    double L = 0;
    for (size_t i = 1; i < pts.size(); ++i) L += (pts[i] - pts[i - 1]).norm();
    return L;
}

void saveCsv(const std::vector<Eigen::Vector3d>& pts, const std::string& file) {
    std::ofstream f(file);
    f << "x,y,z\n";
    for (const auto& p : pts) f << p.x() << "," << p.y() << "," << p.z() << "\n";
}