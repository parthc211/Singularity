#include "Renderer/MeshLoader.h"
#include "Core/Logger.h"

#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace SGE {

// --- internal helpers --------------------------------------------------------

// Parses one face-vertex token of the form "p[/[t][/n]]".
// All returned indices are 1-based (0 means absent).
static void parseFaceToken(const std::string& tok, int& outP, int& outT, int& outN)
{
    outP = outT = outN = 0;
    const char* s = tok.c_str();

    outP = std::atoi(s);

    const char* slash1 = std::strchr(s, '/');
    if (!slash1) return;

    ++slash1; // skip first '/'
    if (*slash1 != '/' && *slash1 != '\0')
        outT = std::atoi(slash1);

    const char* slash2 = std::strchr(slash1, '/');
    if (!slash2) return;

    outN = std::atoi(slash2 + 1);
}

// Converts a 1-based (possibly negative) OBJ index to a 0-based array index.
// Returns -1 for absent (raw == 0).
static int resolveIdx(int raw, int total)
{
    if (raw == 0) return -1;
    return raw > 0 ? raw - 1 : total + raw;
}

// Pack three resolved indices into a uint64_t cache key.
// Each value occupies 21 bits; absent (-1) is stored as the sentinel 0.
// Valid 0-based indices are stored as value+1 so they can't collide with the sentinel.
static uint64_t makeKey(int p, int t, int n)
{
    uint64_t pk = uint64_t(p + 1) & 0x1FFFFF;
    uint64_t tk = uint64_t(t + 1) & 0x1FFFFF;
    uint64_t nk = uint64_t(n + 1) & 0x1FFFFF;
    return pk | (tk << 21) | (nk << 42);
}

// --- public API --------------------------------------------------------------

bool LoadOBJ(const char*              path,
             std::vector<MeshVertex>& outVertices,
             std::vector<uint32_t>&   outIndices)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        LogError(std::string("LoadOBJ: cannot open '") + path + "'");
        return false;
    }

    std::vector<std::array<float, 3>> positions;
    std::vector<std::array<float, 3>> normals;
    std::vector<std::array<float, 2>> uvs;

    std::unordered_map<uint64_t, uint32_t> cache;

    std::string line;
    while (std::getline(file, line)) {
        // Strip Windows line endings
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        if (line.empty() || line[0] == '#') continue;

        std::istringstream ss(line);
        std::string token;
        ss >> token;

        if (token == "v") {
            float x = 0, y = 0, z = 0;
            ss >> x >> y >> z;
            positions.push_back({x, y, z});
        }
        else if (token == "vn") {
            float x = 0, y = 0, z = 0;
            ss >> x >> y >> z;
            normals.push_back({x, y, z});
        }
        else if (token == "vt") {
            float u = 0, v = 0;
            ss >> u >> v;
            uvs.push_back({u, v});
        }
        else if (token == "f") {
            std::vector<uint32_t> faceVerts;
            std::string vtok;

            while (ss >> vtok) {
                int rawP, rawT, rawN;
                parseFaceToken(vtok, rawP, rawT, rawN);

                int p = resolveIdx(rawP, int(positions.size()));
                int t = resolveIdx(rawT, int(uvs.size()));
                int n = resolveIdx(rawN, int(normals.size()));

                if (p < 0 || p >= int(positions.size())) continue;

                uint64_t key = makeKey(p, t, n);
                auto it = cache.find(key);
                if (it != cache.end()) {
                    faceVerts.push_back(it->second);
                }
                else {
                    MeshVertex vert = {};

                    const auto& pos = positions[p];
                    vert.position[0] = pos[0];
                    vert.position[1] = pos[1];
                    vert.position[2] = pos[2];

                    if (n >= 0 && n < int(normals.size())) {
                        vert.normal[0] = normals[n][0];
                        vert.normal[1] = normals[n][1];
                        vert.normal[2] = normals[n][2];
                    }

                    if (t >= 0 && t < int(uvs.size())) {
                        vert.texCoord[0] = uvs[t][0];
                        vert.texCoord[1] = uvs[t][1];
                    }

                    uint32_t idx = uint32_t(outVertices.size());
                    cache[key]   = idx;
                    outVertices.push_back(vert);
                    faceVerts.push_back(idx);
                }
            }

            // Fan-triangulate: (0, i, i+1)
            for (size_t i = 1; i + 1 < faceVerts.size(); ++i) {
                outIndices.push_back(faceVerts[0]);
                outIndices.push_back(faceVerts[i]);
                outIndices.push_back(faceVerts[i + 1]);
            }
        }
        // mtllib, usemtl, o, g, s — silently ignored
    }

    if (outVertices.empty()) {
        LogError(std::string("LoadOBJ: no geometry found in '") + path + "'");
        return false;
    }

    // Without UVs there is no tangent space to derive; leave tangents zeroed.
    if (!uvs.empty())
        ComputeTangents(outVertices, outIndices);

    return true;
}

// --- tangent generation --------------------------------------------------------

namespace {
struct F3 { float x, y, z; };

F3    operator+(F3 a, F3 b)  { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
F3    operator-(F3 a, F3 b)  { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
F3    operator*(F3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }
float Dot(F3 a, F3 b)        { return a.x * b.x + a.y * b.y + a.z * b.z; }
F3    Cross(F3 a, F3 b)      { return { a.y * b.z - a.z * b.y,
                                        a.z * b.x - a.x * b.z,
                                        a.x * b.y - a.y * b.x }; }
float Len(F3 a)              { return std::sqrt(Dot(a, a)); }
F3    LoadF3(const float* p) { return { p[0], p[1], p[2] }; }
} // namespace

void ComputeTangents(std::vector<MeshVertex>&     vertices,
                     const std::vector<uint32_t>& indices)
{
    // Accumulators: unnormalized per-vertex tangent and bitangent sums. Larger
    // triangles contribute proportionally more (the un-normalized per-triangle
    // vectors scale with area), which is the weighting we want.
    std::vector<F3> tanAcc(vertices.size(), { 0, 0, 0 });
    std::vector<F3> bitAcc(vertices.size(), { 0, 0, 0 });

    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const uint32_t i0 = indices[i], i1 = indices[i + 1], i2 = indices[i + 2];
        const MeshVertex& v0 = vertices[i0];
        const MeshVertex& v1 = vertices[i1];
        const MeshVertex& v2 = vertices[i2];

        const F3 e1 = LoadF3(v1.position) - LoadF3(v0.position);
        const F3 e2 = LoadF3(v2.position) - LoadF3(v0.position);

        const float du1 = v1.texCoord[0] - v0.texCoord[0];
        const float dv1 = v1.texCoord[1] - v0.texCoord[1];
        const float du2 = v2.texCoord[0] - v0.texCoord[0];
        const float dv2 = v2.texCoord[1] - v0.texCoord[1];

        // Solve [e1; e2] = [du1 dv1; du2 dv2] * [T; B] for T and B.
        const float det = du1 * dv2 - du2 * dv1;
        if (std::fabs(det) < 1e-12f) continue;  // degenerate UV mapping
        const float r = 1.0f / det;

        const F3 t = (e1 * dv2 - e2 * dv1) * r;
        const F3 b = (e2 * du1 - e1 * du2) * r;

        for (uint32_t idx : { i0, i1, i2 }) {
            tanAcc[idx] = tanAcc[idx] + t;
            bitAcc[idx] = bitAcc[idx] + b;
        }
    }

    for (size_t i = 0; i < vertices.size(); ++i) {
        MeshVertex& v = vertices[i];
        const F3 n = LoadF3(v.normal);
        // Gram-Schmidt: remove the normal component, keeping T in the surface plane.
        F3 t = tanAcc[i] - n * Dot(n, tanAcc[i]);

        float len = Len(t);
        if (len < 1e-8f) {
            // No usable UV-derived tangent (unreferenced vertex, degenerate UVs,
            // or T parallel to N). Build one perpendicular to the normal from
            // the world axis least aligned with it.
            const F3 axis = (std::fabs(n.x) < 0.9f) ? F3{ 1, 0, 0 } : F3{ 0, 1, 0 };
            t   = Cross(n, axis);
            len = Len(t);
            if (len < 1e-8f) { t = { 1, 0, 0 }; len = 1.0f; } // zero normal too
        }
        t = t * (1.0f / len);

        v.tangent[0] = t.x;
        v.tangent[1] = t.y;
        v.tangent[2] = t.z;
        // Handedness: does the accumulated bitangent agree with cross(N, T)?
        v.tangent[3] = (Dot(Cross(n, t), bitAcc[i]) < 0.0f) ? -1.0f : 1.0f;
    }
}

} // namespace SGE
