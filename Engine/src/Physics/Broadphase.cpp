#include "Physics/Broadphase.h"

#include <algorithm>
#include <cmath>

namespace SGE::Physics {

using namespace SGE::Math;

namespace {

int CellCoord(float v, float cellSize) {
    return int(std::floor(v / cellSize));
}

// 21 signed bits per axis packed into one uint64 (range +-2^20 cells — with
// 2 m cells that is +-2000 km, far beyond the camera's far plane).
uint64_t CellKey(int x, int y, int z) {
    auto u = [](int v) { return uint64_t(uint32_t(v + (1 << 20)) & 0x1FFFFF); };
    return u(x) | (u(y) << 21) | (u(z) << 42);
}

} // namespace

bool AABBOverlap(const AABB& a, const AABB& b) {
    float amin[4], amax[4], bmin[4], bmax[4];
    a.Min.Store(amin); a.Max.Store(amax);
    b.Min.Store(bmin); b.Max.Store(bmax);
    for (int k = 0; k < 3; ++k)
        if (amin[k] > bmax[k] || bmin[k] > amax[k]) return false;
    return true;
}

AABB ComputeAABB(const RigidBody& b) {
    switch (b.Shape.Type) {
    case ShapeType::Sphere: {
        Vec4 r(b.Shape.Radius, b.Shape.Radius, b.Shape.Radius, 0.0f);
        return { b.Position - r, b.Position + r };
    }
    case ShapeType::Box: {
        Mat4 R = ToMatrix(b.Orientation);
        float he[4]; b.Shape.Extents.Store(he);
        Vec4 e = Abs(Vec4(R.r[0])) * he[0]
               + Abs(Vec4(R.r[1])) * he[1]
               + Abs(Vec4(R.r[2])) * he[2];
        return { b.Position - e, b.Position + e };
    }
    case ShapeType::Plane:
    default: {
        // Planes are infinite; they never enter the broadphase.
        Vec4 inf(1e30f, 1e30f, 1e30f, 0.0f);
        return { -inf, inf };
    }
    }
}

void Broadphase::CollectPairs(const std::vector<RigidBody>& bodies,
                              const std::vector<AABB>& aabbs,
                              const std::vector<uint32_t>& indices,
                              std::vector<uint64_t>& outPairs) {
    m_cells.clear();
    m_minCell.assign(bodies.size() * 3, 0);

    // --- insert every body into each cell its AABB overlaps ---
    for (uint32_t idx : indices) {
        float mn[4], mx[4];
        aabbs[idx].Min.Store(mn);
        aabbs[idx].Max.Store(mx);

        int c0[3], c1[3];
        for (int k = 0; k < 3; ++k) {
            c0[k] = CellCoord(mn[k], CellSize);
            c1[k] = CellCoord(mx[k], CellSize);
            m_minCell[idx * 3 + k] = c0[k];
        }
        for (int x = c0[0]; x <= c1[0]; ++x)
            for (int y = c0[1]; y <= c1[1]; ++y)
                for (int z = c0[2]; z <= c1[2]; ++z) {
                    Cell& cell = m_cells[CellKey(x, y, z)];
                    if (cell.Bodies.empty()) { cell.x = x; cell.y = y; cell.z = z; }
                    cell.Bodies.push_back(idx);
                }
    }

    // --- emit pairs, deduplicated by the min-common-cell rule ---
    for (const auto& [key, cell] : m_cells) {
        const auto& list = cell.Bodies;
        for (size_t p = 0; p + 1 < list.size(); ++p) {
            for (size_t q = p + 1; q < list.size(); ++q) {
                uint32_t i = list[p], j = list[q];
                if (bodies[i].IsStatic() && bodies[j].IsStatic()) continue;

                // Only the min common cell of the two AABBs reports the pair.
                bool isMinCommon = true;
                const int cc[3] = { cell.x, cell.y, cell.z };
                for (int k = 0; k < 3; ++k)
                    isMinCommon &= cc[k] == std::max(m_minCell[i * 3 + k], m_minCell[j * 3 + k]);
                if (!isMinCommon) continue;

                if (!AABBOverlap(aabbs[i], aabbs[j])) continue;

                const uint64_t lo = std::min(i, j), hi = std::max(i, j);
                outPairs.push_back((lo << 32) | hi);
            }
        }
    }

    // Hash-map iteration order is arbitrary; sorting makes narrowphase and
    // solve order (and therefore the whole trajectory) reproducible.
    std::sort(outPairs.begin(), outPairs.end());
}

} // namespace SGE::Physics
