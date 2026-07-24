#pragma once
// ---------------------------------------------------------------------------
// Broadphase — uniform spatial hash grid, rebuilt from scratch every substep.
//
// Rebuilding is O(n) and keeps the structure trivially correct under fully
// dynamic scenes (no incremental update bugs); the AABB pass is the parallel
// part (per-body, done by the caller via JobSystem), insertion and pair
// collection stay serial so the pair list is deterministic.
//
// Each body is inserted into every cell its AABB overlaps. A pair could then
// be reported once per shared cell, so it is emitted only from the pair's
// "min common cell" (component-wise max of the two AABBs' min cell coords) —
// the standard O(1) dedup that needs no hash set of pairs.
//
// Static planes never enter the grid (an infinite AABB would touch every
// cell); PhysicsWorld tests them against all dynamic bodies directly.
// ---------------------------------------------------------------------------
#include "RigidBody.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace SGE::Physics {

struct AABB {
    Math::Vec4 Min, Max;
};

// World-space bounds. Sphere: pos +- r. Box: pos +- sum |R row k| * halfExt[k]
// (each rotation row is one world-space box axis).
AABB ComputeAABB(const RigidBody& b);

bool AABBOverlap(const AABB& a, const AABB& b);

class Broadphase {
public:
    float CellSize = 2.0f;

    // Inserts bodies listed in `indices` (their aabbs[] entries must be valid)
    // and appends the deduplicated candidate pairs, packed (lo << 32 | hi) and
    // sorted, so the caller's narrowphase/solve order is deterministic.
    // Static-static pairs are skipped at the source.
    void CollectPairs(const std::vector<RigidBody>& bodies,
                      const std::vector<AABB>& aabbs,
                      const std::vector<uint32_t>& indices,
                      std::vector<uint64_t>& outPairs);

private:
    struct Cell {
        int x, y, z;
        std::vector<uint32_t> Bodies;
    };
    std::unordered_map<uint64_t, Cell> m_cells;
    std::vector<int>                   m_minCell; // 3 ints per body (indexed *3)
};

} // namespace SGE::Physics
