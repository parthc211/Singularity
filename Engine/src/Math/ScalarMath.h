#pragma once
// Plain scalar reference math — the honest non-SIMD baseline. Same conventions as
// SimdMath.h (row-major, row-vector; quats stored x,y,z,w). Used both to validate
// the SIMD code (results must match within epsilon) and as the slow contender in
// the benchmark.
#include <cmath>

namespace SGE::Math::Scalar {

struct Vec4 { float x, y, z, w; };
struct Mat4 { float m[4][4]; }; // row-major

inline Vec4 Cross3(const Vec4& a, const Vec4& b) {
    return { a.y*b.z - a.z*b.y,
             a.z*b.x - a.x*b.z,
             a.x*b.y - a.y*b.x,
             0.0f };
}

inline Vec4 Normalize(const Vec4& a) {
    float len = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z + a.w*a.w);
    return { a.x/len, a.y/len, a.z/len, a.w/len };
}

inline Mat4 Mul(const Mat4& A, const Mat4& B) {
    Mat4 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k)
                s += A.m[i][k] * B.m[k][j];
            C.m[i][j] = s;
        }
    return C;
}

// Row-vector transform: result_j = sum_i v_i * M[i][j].
inline Vec4 Transform(const Vec4& v, const Mat4& M) {
    Vec4 r;
    r.x = v.x*M.m[0][0] + v.y*M.m[1][0] + v.z*M.m[2][0] + v.w*M.m[3][0];
    r.y = v.x*M.m[0][1] + v.y*M.m[1][1] + v.z*M.m[2][1] + v.w*M.m[3][1];
    r.z = v.x*M.m[0][2] + v.y*M.m[1][2] + v.z*M.m[2][2] + v.w*M.m[3][2];
    r.w = v.x*M.m[0][3] + v.y*M.m[1][3] + v.z*M.m[2][3] + v.w*M.m[3][3];
    return r;
}

// Quaternion (x,y,z,w) Hamilton product.
inline Vec4 QuatMul(const Vec4& a, const Vec4& b) {
    return {
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
    };
}

// --- element-wise helpers (references for the physics-support SIMD ops) ---
inline Vec4 Add(const Vec4& a, const Vec4& b)  { return { a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w }; }
inline Vec4 Sub(const Vec4& a, const Vec4& b)  { return { a.x-b.x, a.y-b.y, a.z-b.z, a.w-b.w }; }
inline Vec4 Scale(const Vec4& a, float s)      { return { a.x*s, a.y*s, a.z*s, a.w*s }; }
inline Vec4 Neg(const Vec4& a)                 { return { -a.x, -a.y, -a.z, -a.w }; }
inline Vec4 Min(const Vec4& a, const Vec4& b)  { return { std::fmin(a.x,b.x), std::fmin(a.y,b.y), std::fmin(a.z,b.z), std::fmin(a.w,b.w) }; }
inline Vec4 Max(const Vec4& a, const Vec4& b)  { return { std::fmax(a.x,b.x), std::fmax(a.y,b.y), std::fmax(a.z,b.z), std::fmax(a.w,b.w) }; }
inline Vec4 Abs(const Vec4& a)                 { return { std::fabs(a.x), std::fabs(a.y), std::fabs(a.z), std::fabs(a.w) }; }
inline float Dot(const Vec4& a, const Vec4& b) { return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w; }
inline float Dot3(const Vec4& a, const Vec4& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
inline float LengthSq(const Vec4& a)           { return Dot(a, a); }
inline float LengthSq3(const Vec4& a)          { return Dot3(a, a); }

inline Vec4 QuatConjugate(const Vec4& q) { return { -q.x, -q.y, -q.z, q.w }; }

// v' = q v q* expanded to the two-cross form (same identity the SIMD path uses).
inline Vec4 QuatRotate(const Vec4& q, const Vec4& v) {
    Vec4 t = Scale(Cross3(q, v), 2.0f);
    Vec4 r = Add(Add(v, Scale(t, q.w)), Cross3(q, t));
    r.w = v.w;
    return r;
}

// Inverse of the upper 3x3 (cofactor expansion); w row/column identity.
inline Mat4 Inverse3x3(const Mat4& M) {
    const auto& m = M.m;
    float c00 = m[1][1]*m[2][2] - m[1][2]*m[2][1];
    float c01 = m[1][2]*m[2][0] - m[1][0]*m[2][2];
    float c02 = m[1][0]*m[2][1] - m[1][1]*m[2][0];
    float det = m[0][0]*c00 + m[0][1]*c01 + m[0][2]*c02;
    float inv = 1.0f / det;

    Mat4 R{};
    R.m[0][0] = c00 * inv;
    R.m[1][0] = c01 * inv;
    R.m[2][0] = c02 * inv;
    R.m[0][1] = (m[0][2]*m[2][1] - m[0][1]*m[2][2]) * inv;
    R.m[1][1] = (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * inv;
    R.m[2][1] = (m[0][1]*m[2][0] - m[0][0]*m[2][1]) * inv;
    R.m[0][2] = (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * inv;
    R.m[1][2] = (m[0][2]*m[1][0] - m[0][0]*m[1][2]) * inv;
    R.m[2][2] = (m[0][0]*m[1][1] - m[0][1]*m[1][0]) * inv;
    R.m[3][3] = 1.0f;
    return R;
}

} // namespace SGE::Math::Scalar
