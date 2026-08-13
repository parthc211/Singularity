#include "Assets/GltfLoader.h"
#include "Assets/Json.h"
#include "Core/Logger.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <memory>
#include <numeric>

namespace SGE {

using Json::Value;

namespace {

// --- glTF constants ------------------------------------------------------------

constexpr uint32_t kGlbMagic     = 0x46546C67; // "glTF"
constexpr uint32_t kGlbChunkJson = 0x4E4F534A; // "JSON"
constexpr uint32_t kGlbChunkBin  = 0x004E4942; // "BIN\0"

enum ComponentType {
    CT_BYTE = 5120, CT_UBYTE = 5121, CT_SHORT = 5122,
    CT_USHORT = 5123, CT_UINT = 5125, CT_FLOAT = 5126,
};

size_t CompSize(int ct) {
    switch (ct) {
        case CT_BYTE: case CT_UBYTE:   return 1;
        case CT_SHORT: case CT_USHORT: return 2;
        case CT_UINT: case CT_FLOAT:   return 4;
        default:                       return 0;
    }
}

int CompCount(std::string_view type) {
    if (type == "SCALAR") return 1;
    if (type == "VEC2")   return 2;
    if (type == "VEC3")   return 3;
    if (type == "VEC4")   return 4;
    if (type == "MAT4")   return 16;
    return 0; // MAT2/MAT3 unused by anything we read
}

// --- coordinate conversion (RH +Y-up -> LH +Y-up) --------------------------------

Math::Vec4 ConvPos(float x, float y, float z, float w = 0.0f) {
    return Math::Vec4(x, y, -z, w);
}
Math::Quat ConvQuat(float x, float y, float z, float w) {
    // Conjugating a rotation by the Z-mirror negates the axis' x and y parts.
    return Math::Normalize(Math::Quat(-x, -y, z, w));
}
// Same component mapping WITHOUT normalization — for CUBICSPLINE tangents,
// which are derivatives of quaternion components, not unit quaternions.
Math::Quat ConvQuatRaw(float x, float y, float z, float w) {
    return Math::Quat(-x, -y, z, w);
}

// 16 col-major floats (column-vector convention) -> engine row-vector Mat4.
// Reading col-major memory as four row __m128s IS the transpose, which is
// exactly the row-vector form. Then apply C*M*C (C = diag(1,1,-1,1)): negate
// every element with exactly one index equal to 2.
Math::Mat4 ConvMat(const float m[16]) {
    Math::Mat4 r;
    std::memcpy(&r, m, sizeof(r));
    const __m128 flipZcol = _mm_castsi128_ps(_mm_set_epi32(0, 0x80000000, 0, 0));          // lane 2
    const __m128 flipZrow = _mm_castsi128_ps(_mm_set_epi32(0x80000000, 0, 0x80000000, 0x80000000)); // lanes 0,1,3
    r.r[0] = _mm_xor_ps(r.r[0], flipZcol);
    r.r[1] = _mm_xor_ps(r.r[1], flipZcol);
    r.r[2] = _mm_xor_ps(r.r[2], flipZrow);
    r.r[3] = _mm_xor_ps(r.r[3], flipZcol);
    return r;
}

// Decompose an engine-space TRS matrix into a JointPose (for the rare joint
// node that stores `matrix` instead of TRS). Standard row-lengths scale +
// Shepperd quaternion extraction; assumes no shear (true for DCC exports).
Anim::JointPose DecomposeTRS(const Math::Mat4& m) {
    float f[16];
    _mm_storeu_ps(f + 0,  m.r[0]);
    _mm_storeu_ps(f + 4,  m.r[1]);
    _mm_storeu_ps(f + 8,  m.r[2]);
    _mm_storeu_ps(f + 12, m.r[3]);

    Anim::JointPose p;
    p.T = Math::Vec4(f[12], f[13], f[14], 0.0f);

    float sx = std::sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    float sy = std::sqrt(f[4]*f[4] + f[5]*f[5] + f[6]*f[6]);
    float sz = std::sqrt(f[8]*f[8] + f[9]*f[9] + f[10]*f[10]);
    // Negative determinant means one axis is mirrored; fold it into sx.
    const float det = f[0]*(f[5]*f[10]-f[6]*f[9]) - f[1]*(f[4]*f[10]-f[6]*f[8]) + f[2]*(f[4]*f[9]-f[5]*f[8]);
    if (det < 0.0f) sx = -sx;
    p.S = Math::Vec4(sx, sy, sz, 0.0f);

    // Normalized rotation rows.
    const float r[9] = { f[0]/sx, f[1]/sx, f[2]/sx,
                         f[4]/sy, f[5]/sy, f[6]/sy,
                         f[8]/sz, f[9]/sz, f[10]/sz };
    // Row-vector rotation matrix -> quaternion (rows are the rotated basis).
    const float trace = r[0] + r[4] + r[8];
    float qx, qy, qz, qw;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        qw = 0.25f * s;
        qx = (r[5] - r[7]) / s;
        qy = (r[6] - r[2]) / s;
        qz = (r[1] - r[3]) / s;
    } else if (r[0] > r[4] && r[0] > r[8]) {
        const float s = std::sqrt(1.0f + r[0] - r[4] - r[8]) * 2.0f;
        qw = (r[5] - r[7]) / s;
        qx = 0.25f * s;
        qy = (r[3] + r[1]) / s;
        qz = (r[6] + r[2]) / s;
    } else if (r[4] > r[8]) {
        const float s = std::sqrt(1.0f + r[4] - r[0] - r[8]) * 2.0f;
        qw = (r[6] - r[2]) / s;
        qx = (r[3] + r[1]) / s;
        qy = 0.25f * s;
        qz = (r[7] + r[5]) / s;
    } else {
        const float s = std::sqrt(1.0f + r[8] - r[0] - r[4]) * 2.0f;
        qw = (r[1] - r[3]) / s;
        qx = (r[6] + r[2]) / s;
        qy = (r[7] + r[5]) / s;
        qz = 0.25f * s;
    }
    p.R = Math::Normalize(Math::Quat(qx, qy, qz, qw));
    return p;
}

// --- base64 (for data: URIs) ----------------------------------------------------

bool DecodeBase64(std::string_view s, std::vector<uint8_t>& out) {
    auto val = [](char c) -> int {
        if (c >= 'A' && c <= 'Z') return c - 'A';
        if (c >= 'a' && c <= 'z') return c - 'a' + 26;
        if (c >= '0' && c <= '9') return c - '0' + 52;
        if (c == '+') return 62;
        if (c == '/') return 63;
        return -1;
    };
    out.clear();
    out.reserve(s.size() / 4 * 3);
    uint32_t acc = 0;
    int bits = 0;
    for (char c : s) {
        if (c == '=' || c == '\n' || c == '\r') continue;
        const int v = val(c);
        if (v < 0) return false;
        acc = (acc << 6) | uint32_t(v);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(uint8_t((acc >> bits) & 0xFF));
        }
    }
    return true;
}

std::string PercentDecode(std::string_view s) {
    std::string out;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '%' && i + 2 < s.size()) {
            const auto hex = [](char c) -> int {
                if (c >= '0' && c <= '9') return c - '0';
                if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                return -1;
            };
            const int h = hex(s[i + 1]), l = hex(s[i + 2]);
            if (h >= 0 && l >= 0) { out += char(h * 16 + l); i += 2; continue; }
        }
        out += s[i];
    }
    return out;
}

// --- parsed-file context ---------------------------------------------------------

struct Gltf {
    Value root;
    std::vector<std::vector<uint8_t>> buffers;

    // Backing store for materialized accessors (sparse / zero-filled): AccView
    // pointers reference into these, so element addresses must stay stable —
    // hence unique_ptr. Mutable so the const helper chain can lazily add to it.
    mutable std::vector<std::unique_ptr<std::vector<uint8_t>>> materialized;

    const Value* accessors   = nullptr;
    const Value* bufferViews = nullptr;
    const Value* nodes       = nullptr;

    const Value* Arr(const char* key) const {
        const Value* v = root.Find(key);
        return v && v->IsArray() ? v : nullptr;
    }

    // Resolve a bufferView (+extra offset) to raw bytes; nullptr on any error.
    const uint8_t* ViewBytes(int viewIdx, size_t extraOffset, size_t needBytes) const {
        if (!bufferViews || viewIdx < 0 || size_t(viewIdx) >= bufferViews->Size())
            return nullptr;
        const Value& view = (*bufferViews)[size_t(viewIdx)];
        const size_t buf = size_t(view.GetNumber("buffer"));
        if (buf >= buffers.size()) return nullptr;
        const size_t off = size_t(view.GetNumber("byteOffset", 0)) + extraOffset;
        if (off + needBytes > buffers[buf].size()) return nullptr;
        return buffers[buf].data() + off;
    }
};

struct AccView {
    const uint8_t* base = nullptr;
    size_t stride = 0, count = 0;
    int    compType = 0, comps = 0;
    bool   normalized = false;

    bool Valid() const { return base != nullptr; }
};

AccView GetAccessor(const Gltf& g, int index) {
    AccView av;
    if (!g.accessors || index < 0 || size_t(index) >= g.accessors->Size()) return av;
    const Value& acc = (*g.accessors)[size_t(index)];

    const int compType = acc.GetInt("componentType");
    const int comps    = CompCount(acc.GetString("type"));
    const size_t count = size_t(acc.GetNumber("count"));
    const size_t elem  = CompSize(compType) * size_t(comps);
    if (elem == 0 || count == 0) return av;

    av.count      = count;
    av.compType   = compType;
    av.comps      = comps;
    av.normalized = acc.GetBool("normalized");

    const Value* sparse = acc.Find("sparse");
    const Value* bvIdx  = acc.Find("bufferView");

    // Fast path: plain view-backed accessor, point straight into the buffer.
    if (!sparse && bvIdx) {
        const size_t bv = size_t(bvIdx->Number);
        if (!g.bufferViews || bv >= g.bufferViews->Size()) return AccView{};
        const Value& view = (*g.bufferViews)[bv];

        const size_t buf = size_t(view.GetNumber("buffer"));
        if (buf >= g.buffers.size()) return AccView{};
        const auto& bytes = g.buffers[buf];

        const size_t viewOff = size_t(view.GetNumber("byteOffset", 0));
        const size_t viewLen = size_t(view.GetNumber("byteLength", 0));
        const size_t accOff  = size_t(acc.GetNumber("byteOffset", 0));
        const size_t stride  = size_t(view.GetNumber("byteStride", 0));

        av.stride = stride ? stride : elem;
        const size_t end = viewOff + accOff + (count - 1) * av.stride + elem;
        if (end > bytes.size() ||
            (viewLen && accOff + (count - 1) * av.stride + elem > viewLen)) {
            LogError("GltfLoader: accessor range exceeds buffer");
            return AccView{};
        }
        av.base = bytes.data() + viewOff + accOff;
        return av;
    }

    // Materialized path: sparse accessors and view-less (all-zero) accessors
    // get tightly-packed owned storage; base data is copied (or zeroed), then
    // sparse substitutions overwrite the listed elements.
    auto data = std::make_unique<std::vector<uint8_t>>(count * elem, uint8_t(0));

    if (bvIdx) {
        const size_t bv = size_t(bvIdx->Number);
        if (!g.bufferViews || bv >= g.bufferViews->Size()) return AccView{};
        const Value& view    = (*g.bufferViews)[bv];
        const size_t buf     = size_t(view.GetNumber("buffer"));
        if (buf >= g.buffers.size()) return AccView{};
        const auto&  bytes   = g.buffers[buf];
        const size_t viewOff = size_t(view.GetNumber("byteOffset", 0));
        const size_t accOff  = size_t(acc.GetNumber("byteOffset", 0));
        const size_t stride0 = size_t(view.GetNumber("byteStride", 0));
        const size_t stride  = stride0 ? stride0 : elem;
        if (viewOff + accOff + (count - 1) * stride + elem > bytes.size()) {
            LogError("GltfLoader: sparse base range exceeds buffer");
            return AccView{};
        }
        const uint8_t* src = bytes.data() + viewOff + accOff;
        for (size_t e = 0; e < count; ++e)
            std::memcpy(data->data() + e * elem, src + e * stride, elem);
    }

    if (sparse) {
        const size_t scount = size_t(sparse->GetNumber("count"));
        const Value* sIdx   = sparse->Find("indices");
        const Value* sVal   = sparse->Find("values");
        if (!sIdx || !sVal || scount == 0) {
            LogError("GltfLoader: malformed sparse accessor");
            return AccView{};
        }
        const int    idxType = sIdx->GetInt("componentType");
        const size_t idxSize = CompSize(idxType);
        const uint8_t* idxBytes = g.ViewBytes(sIdx->GetInt("bufferView", -1),
                                              size_t(sIdx->GetNumber("byteOffset", 0)),
                                              scount * idxSize);
        const uint8_t* valBytes = g.ViewBytes(sVal->GetInt("bufferView", -1),
                                              size_t(sVal->GetNumber("byteOffset", 0)),
                                              scount * elem);
        if (!idxBytes || !valBytes || idxSize == 0) {
            LogError("GltfLoader: sparse accessor data out of range");
            return AccView{};
        }
        for (size_t k = 0; k < scount; ++k) {
            uint32_t target = 0;
            switch (idxType) {
                case CT_UBYTE:  target = idxBytes[k]; break;
                case CT_USHORT: { uint16_t v; std::memcpy(&v, idxBytes + k * 2, 2); target = v; break; }
                case CT_UINT:   { uint32_t v; std::memcpy(&v, idxBytes + k * 4, 4); target = v; break; }
                default: LogError("GltfLoader: bad sparse index type"); return AccView{};
            }
            if (target >= count) {
                LogError("GltfLoader: sparse index out of range");
                return AccView{};
            }
            std::memcpy(data->data() + size_t(target) * elem, valBytes + k * elem, elem);
        }
    }

    av.stride = elem;
    av.base   = data->data();
    g.materialized.push_back(std::move(data));
    return av;
}

// Component `comp` of element `elem` as float (denormalizing per spec).
float ReadF(const AccView& av, size_t elem, int comp) {
    const uint8_t* p = av.base + elem * av.stride + size_t(comp) * CompSize(av.compType);
    switch (av.compType) {
        case CT_FLOAT:  { float f;    std::memcpy(&f, p, 4); return f; }
        case CT_UBYTE:  { uint8_t v = *p;                       return av.normalized ? v / 255.0f    : float(v); }
        case CT_BYTE:   { int8_t v;   std::memcpy(&v, p, 1);   return av.normalized ? std::max(v / 127.0f, -1.0f) : float(v); }
        case CT_USHORT: { uint16_t v; std::memcpy(&v, p, 2);   return av.normalized ? v / 65535.0f  : float(v); }
        case CT_SHORT:  { int16_t v;  std::memcpy(&v, p, 2);   return av.normalized ? std::max(v / 32767.0f, -1.0f) : float(v); }
        case CT_UINT:   { uint32_t v; std::memcpy(&v, p, 4);   return float(v); }
        default:        return 0.0f;
    }
}

uint32_t ReadU(const AccView& av, size_t elem, int comp) {
    const uint8_t* p = av.base + elem * av.stride + size_t(comp) * CompSize(av.compType);
    switch (av.compType) {
        case CT_UBYTE:  { return *p; }
        case CT_USHORT: { uint16_t v; std::memcpy(&v, p, 2); return v; }
        case CT_UINT:   { uint32_t v; std::memcpy(&v, p, 4); return v; }
        default:        return uint32_t(ReadF(av, elem, comp));
    }
}

// --- file & buffer loading -------------------------------------------------------

bool ReadFileBytes(const std::filesystem::path& path, std::vector<uint8_t>& out) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) return false;
    const std::streamsize size = f.tellg();
    f.seekg(0);
    out.resize(size_t(size));
    return bool(f.read(reinterpret_cast<char*>(out.data()), size));
}

bool LoadBuffers(Gltf& g, const std::filesystem::path& dir, std::vector<uint8_t>&& glbBin) {
    const Value* buffers = g.Arr("buffers");
    if (!buffers) return true; // no buffers: legal, if useless
    g.buffers.resize(buffers->Size());

    for (size_t i = 0; i < buffers->Size(); ++i) {
        const std::string_view uri = (*buffers)[i].GetString("uri");
        if (uri.empty()) {
            // GLB-embedded buffer (only valid for buffer 0).
            if (i == 0 && !glbBin.empty()) { g.buffers[0] = std::move(glbBin); continue; }
            LogError("GltfLoader: buffer has no uri and no GLB BIN chunk");
            return false;
        }
        if (uri.starts_with("data:")) {
            const size_t comma = uri.find(',');
            if (comma == std::string_view::npos ||
                !DecodeBase64(uri.substr(comma + 1), g.buffers[i])) {
                LogError("GltfLoader: bad data: URI buffer");
                return false;
            }
            continue;
        }
        const std::filesystem::path bin = dir / PercentDecode(uri);
        if (!ReadFileBytes(bin, g.buffers[i])) {
            LogError(std::format("GltfLoader: cannot read buffer '{}'", bin.string()));
            return false;
        }
    }
    return true;
}

// --- node helpers -----------------------------------------------------------------

// Local pose of a node in ENGINE space (TRS properties, or decomposed matrix).
Anim::JointPose NodeLocalPose(const Value& node) {
    if (const Value* m = node.Find("matrix"); m && m->IsArray() && m->Size() == 16) {
        float f[16];
        for (int i = 0; i < 16; ++i) f[i] = float((*m)[size_t(i)].Number);
        return DecomposeTRS(ConvMat(f));
    }
    Anim::JointPose p;
    if (const Value* t = node.Find("translation"); t && t->Size() == 3)
        p.T = ConvPos(float((*t)[0].Number), float((*t)[1].Number), float((*t)[2].Number));
    if (const Value* r = node.Find("rotation"); r && r->Size() == 4)
        p.R = ConvQuat(float((*r)[0].Number), float((*r)[1].Number),
                       float((*r)[2].Number), float((*r)[3].Number));
    if (const Value* s = node.Find("scale"); s && s->Size() == 3)
        p.S = Math::Vec4(float((*s)[0].Number), float((*s)[1].Number), float((*s)[2].Number), 0.0f);
    return p;
}

Math::Mat4 PoseMatrix(const Anim::JointPose& p) {
    Math::Mat4 m = Math::ToMatrix(p.R);
    m.r[0] = _mm_mul_ps(m.r[0], _mm_set1_ps(p.S.x()));
    m.r[1] = _mm_mul_ps(m.r[1], _mm_set1_ps(p.S.y()));
    m.r[2] = _mm_mul_ps(m.r[2], _mm_set1_ps(p.S.z()));
    m.r[3] = _mm_set_ps(1.0f, p.T.z(), p.T.y(), p.T.x());
    return m;
}

std::vector<int> BuildNodeParents(const Value& nodes) {
    std::vector<int> parents(nodes.Size(), -1);
    for (size_t n = 0; n < nodes.Size(); ++n) {
        const Value* children = nodes[n].Find("children");
        if (!children || !children->IsArray()) continue;
        for (size_t c = 0; c < children->Size(); ++c) {
            const size_t child = size_t((*children)[c].Number);
            if (child < parents.size()) parents[child] = int(n);
        }
    }
    return parents;
}

// --- vertex attribute extraction ---------------------------------------------------

// Arbitrary unit tangent perpendicular to n (same fallback as the OBJ loader).
void FallbackTangent(const float n[3], float out[4]) {
    const bool useX = std::fabs(n[0]) < 0.9f;
    // cross(n, axis)
    const float ax = useX ? 1.0f : 0.0f, ay = useX ? 0.0f : 1.0f;
    float tx = n[1] * 0.0f - n[2] * ay;
    float ty = n[2] * ax - n[0] * 0.0f;
    float tz = n[0] * ay - n[1] * ax;
    const float len = std::sqrt(tx*tx + ty*ty + tz*tz);
    if (len > 1e-8f) { tx /= len; ty /= len; tz /= len; } else { tx = 1; ty = tz = 0; }
    out[0] = tx; out[1] = ty; out[2] = tz; out[3] = 1.0f;
}

// Appends one primitive's vertices/indices. jointRemap maps JOINTS_0 values
// (positions in skin.joints) to sorted skeleton indices; empty = static mesh.
bool AppendPrimitive(const Gltf& g, const Value& prim,
                     const std::vector<uint32_t>& jointRemap,
                     SkeletalMeshData& out, bool& warnedNoTangent)
{
    if (prim.GetInt("mode", 4) != 4) {
        LogWarn("GltfLoader: skipping non-triangle primitive");
        return true;
    }
    const Value* attrs = prim.Find("attributes");
    if (!attrs) return true;

    const AccView pos = GetAccessor(g, attrs->GetInt("POSITION", -1));
    if (!pos.Valid() || pos.compType != CT_FLOAT || pos.comps < 3) {
        LogError("GltfLoader: primitive POSITION missing or not float VEC3");
        return false;
    }
    const AccView nrm = GetAccessor(g, attrs->GetInt("NORMAL", -1));
    const AccView uv  = GetAccessor(g, attrs->GetInt("TEXCOORD_0", -1));
    const AccView tan = GetAccessor(g, attrs->GetInt("TANGENT", -1));
    const AccView jnt = GetAccessor(g, attrs->GetInt("JOINTS_0", -1));
    const AccView wgt = GetAccessor(g, attrs->GetInt("WEIGHTS_0", -1));

    if (!tan.Valid() && !warnedNoTangent) {
        LogInfo("GltfLoader: no TANGENT attribute — using normal-perpendicular fallback");
        warnedNoTangent = true;
    }

    const uint32_t baseVertex = uint32_t(out.Vertices.size());
    const bool skinned = !jointRemap.empty() && jnt.Valid() && wgt.Valid();

    for (size_t v = 0; v < pos.count; ++v) {
        SkinnedVertex sv = {};
        sv.position[0] =  ReadF(pos, v, 0);
        sv.position[1] =  ReadF(pos, v, 1);
        sv.position[2] = -ReadF(pos, v, 2);

        if (nrm.Valid() && nrm.comps >= 3) {
            sv.normal[0] =  ReadF(nrm, v, 0);
            sv.normal[1] =  ReadF(nrm, v, 1);
            sv.normal[2] = -ReadF(nrm, v, 2);
        } else {
            sv.normal[1] = 1.0f;
        }

        if (uv.Valid() && uv.comps >= 2) {
            sv.texCoord[0] = ReadF(uv, v, 0);
            sv.texCoord[1] = ReadF(uv, v, 1);
        }

        if (tan.Valid() && tan.comps >= 4) {
            sv.tangent[0] =  ReadF(tan, v, 0);
            sv.tangent[1] =  ReadF(tan, v, 1);
            sv.tangent[2] = -ReadF(tan, v, 2);
            sv.tangent[3] = -ReadF(tan, v, 3);   // mirror flips handedness
        } else {
            FallbackTangent(sv.normal, sv.tangent);
        }

        if (skinned) {
            float w[4] = { ReadF(wgt, v, 0), ReadF(wgt, v, 1),
                           ReadF(wgt, v, 2), ReadF(wgt, v, 3) };
            const float sum = w[0] + w[1] + w[2] + w[3];
            if (sum > 1e-6f) {
                for (int k = 0; k < 4; ++k) w[k] /= sum;
            } else {
                w[0] = 1.0f; w[1] = w[2] = w[3] = 0.0f;
            }
            for (int k = 0; k < 4; ++k) {
                const uint32_t j = ReadU(jnt, v, k);
                sv.joints[k]  = j < jointRemap.size() ? jointRemap[j] : 0;
                sv.weights[k] = w[k];
            }
        } else {
            sv.weights[0] = 1.0f;   // full weight on joint 0 (identity for static)
        }
        out.Vertices.push_back(sv);
    }

    // Indices: REWIND each triangle (swap its 2nd/3rd corner). The Z-mirror
    // reverses triangle orientation, but moving from glTF's right-handed view
    // convention to the engine's left-handed one reverses apparent orientation
    // AGAIN — so imported CCW front faces would still appear CCW on screen and
    // the rasterizer (CW = front) would cull them, rendering meshes inside-out.
    // The swap restores clockwise front faces. (Regression: gltf_test asserts
    // the imported mesh's signed volume matches cube.obj's winding convention.)
    const Value* idxV = prim.Find("indices");
    if (idxV) {
        const AccView idx = GetAccessor(g, int(idxV->Number));
        if (!idx.Valid()) { LogError("GltfLoader: bad index accessor"); return false; }
        for (size_t i = 0; i + 2 < idx.count; i += 3) {
            out.Indices.push_back(baseVertex + ReadU(idx, i + 0, 0));
            out.Indices.push_back(baseVertex + ReadU(idx, i + 2, 0));
            out.Indices.push_back(baseVertex + ReadU(idx, i + 1, 0));
        }
    } else {
        for (size_t i = 0; i + 2 < pos.count; i += 3) {
            out.Indices.push_back(baseVertex + uint32_t(i + 0));
            out.Indices.push_back(baseVertex + uint32_t(i + 2));
            out.Indices.push_back(baseVertex + uint32_t(i + 1));
        }
    }
    return true;
}

// --- material texture extraction ----------------------------------------------------

// Decode images[imageIndex] to RGBA8, whichever way the file stores it:
// a bufferView (GLB-embedded PNG/JPEG), a base64 data URI, or an external file.
bool DecodeGltfImage(const Gltf& g, int imageIndex,
                     const std::filesystem::path& dir, Image& out)
{
    const Value* images = g.Arr("images");
    if (!images || imageIndex < 0 || size_t(imageIndex) >= images->Size())
        return false;
    const Value& img = (*images)[size_t(imageIndex)];

    if (const Value* bv = img.Find("bufferView")) {
        if (!g.bufferViews || size_t(bv->Number) >= g.bufferViews->Size())
            return false;
        const Value& view = (*g.bufferViews)[size_t(bv->Number)];
        const size_t buf  = size_t(view.GetNumber("buffer"));
        const size_t off  = size_t(view.GetNumber("byteOffset", 0));
        const size_t len  = size_t(view.GetNumber("byteLength", 0));
        if (buf >= g.buffers.size() || len == 0 || off + len > g.buffers[buf].size())
            return false;
        return LoadImageFromMemoryRGBA8(g.buffers[buf].data() + off, len, out);
    }

    const std::string_view uri = img.GetString("uri");
    if (uri.empty()) return false;
    if (uri.starts_with("data:")) {
        const size_t comma = uri.find(',');
        std::vector<uint8_t> bytes;
        if (comma == std::string_view::npos ||
            !DecodeBase64(uri.substr(comma + 1), bytes))
            return false;
        return LoadImageFromMemoryRGBA8(bytes.data(), bytes.size(), out);
    }
    const std::filesystem::path file = dir / PercentDecode(uri);
    return LoadImageRGBA8(file.c_str(), out);
}

// textures[textureIndex] -> images[] index (-1 when absent).
int TextureImageIndex(const Gltf& g, int textureIndex)
{
    const Value* textures = g.Arr("textures");
    if (!textures || textureIndex < 0 || size_t(textureIndex) >= textures->Size())
        return -1;
    return (*textures)[size_t(textureIndex)].GetInt("source", -1);
}

// Pull base-color + normal textures (and the base-color factor) of the mesh's
// first material-bearing primitive into `out`.
void LoadMaterial(const Gltf& g, const Value& prims,
                  const std::filesystem::path& dir, SkeletalMeshData& out)
{
    const Value* materials = g.Arr("materials");
    if (!materials) return;

    int materialIdx = -1;
    for (size_t p = 0; p < prims.Size() && materialIdx < 0; ++p)
        materialIdx = prims[p].GetInt("material", -1);
    if (materialIdx < 0 || size_t(materialIdx) >= materials->Size()) return;
    const Value& mat = (*materials)[size_t(materialIdx)];

    if (const Value* pbr = mat.Find("pbrMetallicRoughness")) {
        if (const Value* f = pbr->Find("baseColorFactor"); f && f->Size() == 4)
            for (int k = 0; k < 4; ++k)
                out.BaseColorFactor[k] = float((*f)[size_t(k)].Number);
        if (const Value* t = pbr->Find("baseColorTexture"))
            DecodeGltfImage(g, TextureImageIndex(g, t->GetInt("index", -1)),
                            dir, out.BaseColorImage);
    }
    if (const Value* nt = mat.Find("normalTexture"))
        DecodeGltfImage(g, TextureImageIndex(g, nt->GetInt("index", -1)),
                        dir, out.NormalImage);
}

// --- animation extraction -----------------------------------------------------------

// Copies one sampler's keys into a channel, exactly: LINEAR and STEP as-is;
// CUBICSPLINE stores per-key triplets (in-tangent, value, out-tangent), which
// land in the channel's InTan/Values/OutTan for exact Hermite sampling.
// readValue converts an output element to engine space; readTangent converts a
// tangent element (same mapping WITHOUT normalization — tangents are not unit).
template <typename Channel, typename ReadValueFn, typename ReadTangentFn>
bool ReadSampler(const Gltf& g, const Value& sampler, Channel& ch,
                 ReadValueFn&& readValue, ReadTangentFn&& readTangent)
{
    const AccView in  = GetAccessor(g, sampler.GetInt("input", -1));
    const AccView val = GetAccessor(g, sampler.GetInt("output", -1));
    if (!in.Valid() || !val.Valid()) return false;

    const std::string_view interp = sampler.GetString("interpolation", "LINEAR");
    const bool cubic = interp == "CUBICSPLINE";
    ch.Interp = cubic              ? Anim::Interpolation::CubicSpline
              : interp == "STEP"   ? Anim::Interpolation::Step
                                   : Anim::Interpolation::Linear;
    if (cubic && val.count != in.count * 3) return false;
    if (!cubic && val.count < in.count) return false;

    ch.Times.resize(in.count);
    for (size_t k = 0; k < in.count; ++k) {
        ch.Times[k] = ReadF(in, k, 0);
        if (cubic) {
            ch.InTan.push_back(readTangent(val, k * 3 + 0));
            ch.Values.push_back(readValue(val, k * 3 + 1));
            ch.OutTan.push_back(readTangent(val, k * 3 + 2));
        } else {
            ch.Values.push_back(readValue(val, k));
        }
    }
    return true;
}

void LoadAnimations(const Gltf& g, const std::vector<int>& nodeToJoint,
                    uint32_t jointCount, std::vector<Anim::AnimationClip>& outClips)
{
    const Value* animations = g.Arr("animations");
    if (!animations) return;

    const auto readVec3 = [](const AccView& v, size_t k) {
        return ConvPos(ReadF(v, k, 0), ReadF(v, k, 1), ReadF(v, k, 2));
    };
    const auto readScale = [](const AccView& v, size_t k) {
        return Math::Vec4(ReadF(v, k, 0), ReadF(v, k, 1), ReadF(v, k, 2), 0.0f);
    };
    const auto readQuat = [](const AccView& v, size_t k) {
        return ConvQuat(ReadF(v, k, 0), ReadF(v, k, 1), ReadF(v, k, 2), ReadF(v, k, 3));
    };
    const auto readQuatTan = [](const AccView& v, size_t k) {
        return ConvQuatRaw(ReadF(v, k, 0), ReadF(v, k, 1), ReadF(v, k, 2), ReadF(v, k, 3));
    };

    for (size_t a = 0; a < animations->Size(); ++a) {
        const Value& anim = (*animations)[a];
        const Value* channels = anim.Find("channels");
        const Value* samplers = anim.Find("samplers");
        if (!channels || !samplers) continue;

        Anim::AnimationClip clip;
        clip.Name = std::string(anim.GetString("name", std::format("clip{}", a)));
        clip.Tracks.resize(jointCount);

        for (size_t c = 0; c < channels->Size(); ++c) {
            const Value& channel = (*channels)[c];
            const Value* target  = channel.Find("target");
            if (!target) continue;
            const int node = target->GetInt("node", -1);
            if (node < 0 || size_t(node) >= nodeToJoint.size() || nodeToJoint[node] < 0)
                continue;   // channel drives a non-joint node — out of scope
            const int samplerIdx = channel.GetInt("sampler", -1);
            if (samplerIdx < 0 || size_t(samplerIdx) >= samplers->Size()) continue;
            const Value& sampler = (*samplers)[size_t(samplerIdx)];

            Anim::JointTrack& track = clip.Tracks[size_t(nodeToJoint[node])];
            const std::string_view path = target->GetString("path");

            bool ok = true;
            if (path == "translation") {
                ok = ReadSampler(g, sampler, track.Translation, readVec3, readVec3);
                if (!ok) track.Translation = {};
            } else if (path == "rotation") {
                ok = ReadSampler(g, sampler, track.Rotation, readQuat, readQuatTan);
                if (!ok) track.Rotation = {};
            } else if (path == "scale") {
                ok = ReadSampler(g, sampler, track.Scale, readScale, readScale);
                if (!ok) track.Scale = {};
            }
            // "weights" (morph targets) fall through: unsupported.

            if (!ok) LogWarn("GltfLoader: dropped malformed animation channel");
        }

        for (const Anim::JointTrack& t : clip.Tracks) {
            if (!t.Translation.Times.empty()) clip.Duration = std::max(clip.Duration, t.Translation.Times.back());
            if (!t.Rotation.Times.empty())    clip.Duration = std::max(clip.Duration, t.Rotation.Times.back());
            if (!t.Scale.Times.empty())       clip.Duration = std::max(clip.Duration, t.Scale.Times.back());
        }
        if (clip.Duration > 0.0f)
            outClips.push_back(std::move(clip));
    }
}

} // namespace

// --- public entry point ---------------------------------------------------------------

bool LoadGLTF(const char* path, SkeletalMeshData& out)
{
    out = {};
    const std::filesystem::path file(path);

    std::vector<uint8_t> raw;
    if (!ReadFileBytes(file, raw)) {
        LogError(std::format("LoadGLTF: cannot open '{}'", path));
        return false;
    }

    // --- container: GLB or plain JSON ---
    std::string jsonText;
    std::vector<uint8_t> glbBin;
    if (raw.size() >= 12 && std::memcmp(raw.data(), &kGlbMagic, 4) == 0) {
        uint32_t version;
        std::memcpy(&version, raw.data() + 4, 4);
        if (version != 2) {
            LogError("LoadGLTF: unsupported GLB version");
            return false;
        }
        size_t offset = 12;
        while (offset + 8 <= raw.size()) {
            uint32_t chunkLen, chunkType;
            std::memcpy(&chunkLen,  raw.data() + offset,     4);
            std::memcpy(&chunkType, raw.data() + offset + 4, 4);
            offset += 8;
            if (offset + chunkLen > raw.size()) break;
            if (chunkType == kGlbChunkJson)
                jsonText.assign(reinterpret_cast<const char*>(raw.data() + offset), chunkLen);
            else if (chunkType == kGlbChunkBin)
                glbBin.assign(raw.data() + offset, raw.data() + offset + chunkLen);
            offset += (size_t(chunkLen) + 3) & ~size_t(3);
        }
        if (jsonText.empty()) {
            LogError("LoadGLTF: GLB has no JSON chunk");
            return false;
        }
    } else {
        jsonText.assign(reinterpret_cast<const char*>(raw.data()), raw.size());
    }

    Gltf g;
    std::string jsonError;
    if (!Json::Parse(jsonText, g.root, &jsonError)) {
        LogError(std::format("LoadGLTF: {} — {}", path, jsonError));
        return false;
    }
    g.accessors   = g.Arr("accessors");
    g.bufferViews = g.Arr("bufferViews");
    g.nodes       = g.Arr("nodes");
    if (!g.nodes) {
        LogError("LoadGLTF: no nodes");
        return false;
    }
    if (!LoadBuffers(g, file.parent_path(), std::move(glbBin)))
        return false;

    const std::vector<int> nodeParents = BuildNodeParents(*g.nodes);

    // --- pick the node to import: first skinned node, else first mesh node ---
    int meshNode = -1, skinIdx = -1;
    for (size_t n = 0; n < g.nodes->Size(); ++n) {
        const Value& node = (*g.nodes)[n];
        if (node.Find("mesh") && node.Find("skin")) {
            meshNode = int(n);
            skinIdx  = node.GetInt("skin");
            break;
        }
    }
    if (meshNode < 0) {
        for (size_t n = 0; n < g.nodes->Size(); ++n)
            if ((*g.nodes)[n].Find("mesh")) { meshNode = int(n); break; }
    }
    if (meshNode < 0) {
        LogError("LoadGLTF: no mesh in file");
        return false;
    }

    // --- skeleton (or a single identity joint for static meshes) ---
    std::vector<uint32_t> jointRemap;             // skin.joints position -> skeleton index
    std::vector<int>      nodeToJoint(g.nodes->Size(), -1);

    if (skinIdx >= 0) {
        const Value* skins = g.Arr("skins");
        if (!skins || size_t(skinIdx) >= skins->Size()) {
            LogError("LoadGLTF: node references missing skin");
            return false;
        }
        const Value& skin      = (*skins)[size_t(skinIdx)];
        const Value* jointsArr = skin.Find("joints");
        if (!jointsArr || jointsArr->Size() == 0) {
            LogError("LoadGLTF: skin has no joints");
            return false;
        }
        const size_t jointCount = jointsArr->Size();
        if (jointCount > 256) {
            LogError(std::format("LoadGLTF: {} joints exceeds the 256-joint palette", jointCount));
            return false;
        }

        std::vector<int> jointNodes(jointCount);
        for (size_t j = 0; j < jointCount; ++j)
            jointNodes[j] = int((*jointsArr)[j].Number);

        // Sort joints by node depth so parents always precede children.
        std::vector<int> depth(g.nodes->Size(), 0);
        for (size_t n = 0; n < g.nodes->Size(); ++n) {
            int d = 0;
            for (int p = nodeParents[n]; p >= 0; p = nodeParents[size_t(p)]) ++d;
            depth[n] = d;
        }
        std::vector<uint32_t> order(jointCount);
        std::iota(order.begin(), order.end(), 0u);
        std::stable_sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
            return depth[size_t(jointNodes[a])] < depth[size_t(jointNodes[b])];
        });

        // Per spec, joint globals are SCENE-graph globals: nodes ABOVE the root
        // joint still transform the skeleton (CesiumMan parks its Z-up -> Y-up
        // correction on one — dropping it lays the character on the floor).
        // Compose that ancestor prefix and, when it isn't identity, emit it as
        // a fixed pseudo-root joint (never animated: no track ever targets it),
        // so ComputeGlobals stays a pure parent-before-child pass.
        Math::Mat4 prefix = Math::Mat4::Identity();
        for (int p = nodeParents[size_t(jointNodes[order[0]])]; p >= 0;
             p = nodeParents[size_t(p)])
            prefix = Math::Mul(prefix, PoseMatrix(NodeLocalPose((*g.nodes)[size_t(p)])));

        bool hasPrefix = false;
        {
            const Math::Mat4 I = Math::Mat4::Identity();
            float fp[16], fi[16];
            for (int r = 0; r < 4; ++r) {
                _mm_storeu_ps(fp + r * 4, prefix.r[r]);
                _mm_storeu_ps(fi + r * 4, I.r[r]);
            }
            for (int k = 0; k < 16; ++k)
                hasPrefix = hasPrefix || std::fabs(fp[k] - fi[k]) > 1e-5f;
        }
        const uint32_t extra = hasPrefix ? 1u : 0u;
        if (jointCount + extra > 256) {
            LogError("LoadGLTF: joint count exceeds the 256-joint palette");
            return false;
        }

        jointRemap.resize(jointCount);
        for (uint32_t newIdx = 0; newIdx < jointCount; ++newIdx) {
            jointRemap[order[newIdx]] = newIdx + extra;
            nodeToJoint[size_t(jointNodes[order[newIdx]])] = int(newIdx + extra);
        }

        // Inverse bind matrices arrive in skin.joints order.
        const AccView ibm = GetAccessor(g, skin.GetInt("inverseBindMatrices", -1));

        Anim::Skeleton& sk = out.Skeleton;
        const uint32_t total = uint32_t(jointCount) + extra;
        sk.Names.resize(total);
        sk.Parents.resize(total);
        sk.RestPose.resize(total);
        sk.InverseBind.resize(total, Math::Mat4::Identity());

        if (hasPrefix) {
            sk.Names[0]    = "<scene-prefix>";
            sk.Parents[0]  = Anim::kInvalidJoint;
            sk.RestPose[0] = DecomposeTRS(prefix);   // fixed; no clip animates it
        }

        for (uint32_t newIdx = 0; newIdx < jointCount; ++newIdx) {
            const uint32_t dst    = newIdx + extra;
            const uint32_t oldIdx = order[newIdx];
            const int      nodeId = jointNodes[oldIdx];
            const Value&   node   = (*g.nodes)[size_t(nodeId)];

            sk.Names[dst]    = std::string(node.GetString("name", std::format("joint{}", dst)));
            sk.RestPose[dst] = NodeLocalPose(node);

            // Parent = nearest ancestor node that is also a joint; true roots
            // hang off the pseudo-root when there is one.
            int p = nodeParents[size_t(nodeId)];
            while (p >= 0 && nodeToJoint[size_t(p)] < 0) p = nodeParents[size_t(p)];
            sk.Parents[dst] = p >= 0        ? uint32_t(nodeToJoint[size_t(p)])
                            : hasPrefix     ? 0u
                                            : Anim::kInvalidJoint;

            if (ibm.Valid() && ibm.comps == 16 && oldIdx < ibm.count) {
                float m[16];
                for (int k = 0; k < 16; ++k) m[k] = ReadF(ibm, oldIdx, k);
                sk.InverseBind[dst] = ConvMat(m);
            }
        }
    } else {
        // Static fallback: one identity joint so the skinned pipeline still works.
        Anim::Skeleton& sk = out.Skeleton;
        sk.Names       = { "root" };
        sk.Parents     = { Anim::kInvalidJoint };
        sk.RestPose.resize(1);
        sk.InverseBind = { Math::Mat4::Identity() };
    }

    // --- geometry ---
    const Value* meshes  = g.Arr("meshes");
    const int    meshIdx = (*g.nodes)[size_t(meshNode)].GetInt("mesh");
    if (!meshes || size_t(meshIdx) >= meshes->Size()) {
        LogError("LoadGLTF: bad mesh index");
        return false;
    }
    const Value* prims = (*meshes)[size_t(meshIdx)].Find("primitives");
    if (!prims || !prims->IsArray() || prims->Size() == 0) {
        LogError("LoadGLTF: mesh has no primitives");
        return false;
    }
    bool warnedNoTangent = false;
    for (size_t p = 0; p < prims->Size(); ++p) {
        if (!AppendPrimitive(g, (*prims)[p], jointRemap, out, warnedNoTangent))
            return false;
    }

    // Skinned vertices are authored in model space (the node transform is
    // ignored per spec). For the static fallback, bake the node's global
    // transform so the mesh lands where the file put it.
    if (skinIdx < 0) {
        Math::Mat4 global = PoseMatrix(NodeLocalPose((*g.nodes)[size_t(meshNode)]));
        for (int p = nodeParents[size_t(meshNode)]; p >= 0; p = nodeParents[size_t(p)])
            global = Math::Mul(global, PoseMatrix(NodeLocalPose((*g.nodes)[size_t(p)])));
        for (SkinnedVertex& v : out.Vertices) {
            const Math::Vec4 pos = Math::Transform(
                Math::Vec4(v.position[0], v.position[1], v.position[2], 1.0f), global);
            v.position[0] = pos.x(); v.position[1] = pos.y(); v.position[2] = pos.z();
            const Math::Vec4 n = Math::Normalize3(Math::Transform(
                Math::Vec4(v.normal[0], v.normal[1], v.normal[2], 0.0f), global));
            v.normal[0] = n.x(); v.normal[1] = n.y(); v.normal[2] = n.z();
        }
    }

    // --- material textures ---
    LoadMaterial(g, *prims, file.parent_path(), out);

    // --- animations ---
    LoadAnimations(g, nodeToJoint, out.Skeleton.JointCount(), out.Clips);

    LogInfo(std::format("LoadGLTF: '{}' — {} verts, {} tris, {} joints, {} clip(s), albedo {}",
                        path, out.Vertices.size(), out.Indices.size() / 3,
                        out.Skeleton.JointCount(), out.Clips.size(),
                        out.BaseColorImage.IsValid()
                            ? std::format("{}x{}", out.BaseColorImage.width, out.BaseColorImage.height)
                            : "none"));
    return true;
}

} // namespace SGE
