#include "Renderer/MeshLoader.h"
#include "Core/Logger.h"

#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include <vector>
#include <unordered_map>
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

    return true;
}

} // namespace SGE
