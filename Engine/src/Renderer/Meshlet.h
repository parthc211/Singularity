#pragma once

#include "Renderer/Mesh.h"   // MeshVertex

#include <cstdint>
#include <vector>

namespace SGE {

// Meshlet rendering data. A mesh is partitioned into small clusters ("meshlets")
// of at most kMaxMeshletVerts unique vertices and kMaxMeshletPrims triangles. The
// mesh-shader front end launches one thread group per meshlet; an amplification
// shader first culls whole meshlets (frustum + normal cone) before dispatching the
// survivors. This is the Nanite-adjacent geometry pipeline: culling and LOD happen
// per cluster on the GPU instead of per draw on the CPU.
//
// Limits: a mesh-shader group may output at most 256 vertices / 256 primitives, so
// 64 / 124 sits comfortably inside and matches common IHV guidance.
constexpr uint32_t kMaxMeshletVerts = 64;
constexpr uint32_t kMaxMeshletPrims = 124;

// One cluster: a window into the shared vertexIndices / primIndices arrays.
struct Meshlet {
    uint32_t vertOffset;   // first entry in MeshletMesh::vertexIndices
    uint32_t vertCount;    // unique vertices, <= kMaxMeshletVerts
    uint32_t primOffset;   // first entry in MeshletMesh::primIndices
    uint32_t primCount;    // triangles, <= kMaxMeshletPrims
};

// Per-meshlet culling data (object space). The normal cone enables whole-cluster
// back-face rejection: with axis = average face normal and cutoff = sin(spread),
// the cluster is entirely back-facing when
//     dot(coneAxis, normalize(center - cameraPos)) > coneCutoff.
// coneCutoff > 1 (we store 2) disables the test for clusters whose normals spread
// past 90° (a cone that wide can never be proven fully back-facing).
struct MeshletBounds {
    float center[3];     float radius;      // bounding sphere
    float coneAxis[3];   float coneCutoff;  // normal cone (see above)
};

// The partitioned mesh. vertexIndices holds each meshlet's unique GLOBAL vertex
// indices (into the source vertex buffer), grouped by meshlet. primIndices holds
// its triangles, one uint per triangle with the three LOCAL vertex indices
// (0..vertCount-1) packed 10 bits each: i0 | (i1<<10) | (i2<<20).
struct MeshletMesh {
    std::vector<Meshlet>       meshlets;
    std::vector<MeshletBounds> bounds;
    std::vector<uint32_t>      vertexIndices;
    std::vector<uint32_t>      primIndices;

    bool     IsValid()      const { return !meshlets.empty(); }
    uint32_t MeshletCount() const { return uint32_t(meshlets.size()); }
};

// Greedily partition an indexed triangle mesh into meshlets and compute per-meshlet
// bounds. Triangles are consumed in order (good enough for the demo meshes); a
// production builder would reorder for spatial locality first.
MeshletMesh BuildMeshlets(const std::vector<MeshVertex>& vertices,
                          const std::vector<uint32_t>&   indices);

} // namespace SGE
