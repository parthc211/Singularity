#include "Renderer/Meshlet.h"

#include <array>
#include <cmath>
#include <unordered_map>

namespace SGE {

namespace {

// Average + max-distance bounding sphere and normal cone for one finished meshlet.
MeshletBounds ComputeBounds(const std::vector<MeshVertex>& verts,
                            const std::vector<uint32_t>&   vertexIndices,
                            const std::vector<uint32_t>&   primIndices,
                            const Meshlet&                 m)
{
    MeshletBounds b = {};

    // Bounding sphere: centroid of the meshlet's unique vertices, then max radius.
    float cx = 0, cy = 0, cz = 0;
    for (uint32_t i = 0; i < m.vertCount; ++i) {
        const MeshVertex& v = verts[vertexIndices[m.vertOffset + i]];
        cx += v.position[0]; cy += v.position[1]; cz += v.position[2];
    }
    const float inv = m.vertCount ? 1.0f / float(m.vertCount) : 0.0f;
    cx *= inv; cy *= inv; cz *= inv;
    float r2 = 0.0f;
    for (uint32_t i = 0; i < m.vertCount; ++i) {
        const MeshVertex& v = verts[vertexIndices[m.vertOffset + i]];
        const float dx = v.position[0]-cx, dy = v.position[1]-cy, dz = v.position[2]-cz;
        r2 = (std::max)(r2, dx*dx + dy*dy + dz*dz);
    }
    b.center[0] = cx; b.center[1] = cy; b.center[2] = cz;
    b.radius    = std::sqrt(r2);

    // Normal cone: axis = normalized sum of per-triangle face normals; spread =
    // the widest angle any face normal makes with the axis.
    float ax = 0, ay = 0, az = 0;
    std::vector<std::array<float,3>> faceN; faceN.reserve(m.primCount);
    for (uint32_t t = 0; t < m.primCount; ++t) {
        const uint32_t packed = primIndices[m.primOffset + t];
        const uint32_t l0 = packed & 0x3FF, l1 = (packed >> 10) & 0x3FF, l2 = (packed >> 20) & 0x3FF;
        const MeshVertex& p0 = verts[vertexIndices[m.vertOffset + l0]];
        const MeshVertex& p1 = verts[vertexIndices[m.vertOffset + l1]];
        const MeshVertex& p2 = verts[vertexIndices[m.vertOffset + l2]];
        const float e1x = p1.position[0]-p0.position[0], e1y = p1.position[1]-p0.position[1], e1z = p1.position[2]-p0.position[2];
        const float e2x = p2.position[0]-p0.position[0], e2y = p2.position[1]-p0.position[1], e2z = p2.position[2]-p0.position[2];
        float nx = e1y*e2z - e1z*e2y, ny = e1z*e2x - e1x*e2z, nz = e1x*e2y - e1y*e2x;
        const float len = std::sqrt(nx*nx + ny*ny + nz*nz);
        if (len > 1e-8f) { nx/=len; ny/=len; nz/=len; }
        faceN.push_back({ nx, ny, nz });
        ax += nx; ay += ny; az += nz;
    }
    const float alen = std::sqrt(ax*ax + ay*ay + az*az);
    if (alen > 1e-6f) {
        ax/=alen; ay/=alen; az/=alen;
        float minDot = 1.0f;
        for (const auto& n : faceN)
            minDot = (std::min)(minDot, ax*n[0] + ay*n[1] + az*n[2]);
        b.coneAxis[0] = ax; b.coneAxis[1] = ay; b.coneAxis[2] = az;
        // spread angle = acos(minDot); cutoff = sin(spread) = sqrt(1 - minDot^2).
        // minDot <= 0 means the cone is >= 90° wide — never provably back-facing,
        // so disable the test with a cutoff the dot product can never exceed.
        b.coneCutoff = (minDot > 0.0f) ? std::sqrt((std::max)(0.0f, 1.0f - minDot*minDot)) : 2.0f;
    } else {
        // Degenerate / fully opposed normals: disable cone culling.
        b.coneAxis[0] = 0; b.coneAxis[1] = 0; b.coneAxis[2] = 1; b.coneCutoff = 2.0f;
    }
    return b;
}

} // namespace

MeshletMesh BuildMeshlets(const std::vector<MeshVertex>& vertices,
                          const std::vector<uint32_t>&   indices)
{
    MeshletMesh out;
    if (vertices.empty() || indices.size() < 3) return out;

    Meshlet cur = {};
    cur.vertOffset = 0;
    cur.primOffset = 0;

    // Maps a GLOBAL vertex index -> its LOCAL index within the current meshlet.
    std::unordered_map<uint32_t, uint32_t> local;
    local.reserve(kMaxMeshletVerts * 2);

    auto finalize = [&]() {
        if (cur.primCount == 0) return;
        out.bounds.push_back(ComputeBounds(vertices, out.vertexIndices, out.primIndices, cur));
        out.meshlets.push_back(cur);
        // Start a fresh meshlet after the data just appended.
        Meshlet next = {};
        next.vertOffset = uint32_t(out.vertexIndices.size());
        next.primOffset = uint32_t(out.primIndices.size());
        cur = next;
        local.clear();
    };

    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const uint32_t tri[3] = { indices[i], indices[i+1], indices[i+2] };

        // How many of this triangle's vertices are not yet in the current meshlet?
        uint32_t fresh = 0;
        for (uint32_t g : tri)
            if (local.find(g) == local.end()) ++fresh;

        // Close the meshlet if the triangle would overflow either limit.
        if (cur.vertCount + fresh > kMaxMeshletVerts || cur.primCount + 1 > kMaxMeshletPrims)
            finalize();

        // Add the triangle, assigning local indices to any new vertices.
        uint32_t localIdx[3];
        for (int k = 0; k < 3; ++k) {
            const uint32_t g = tri[k];
            auto it = local.find(g);
            if (it == local.end()) {
                const uint32_t li = cur.vertCount++;
                local.emplace(g, li);
                out.vertexIndices.push_back(g);
                localIdx[k] = li;
            } else {
                localIdx[k] = it->second;
            }
        }
        out.primIndices.push_back(localIdx[0] | (localIdx[1] << 10) | (localIdx[2] << 20));
        ++cur.primCount;
    }
    finalize();
    return out;
}

} // namespace SGE
