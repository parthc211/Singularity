#pragma once
// ---------------------------------------------------------------------------
// Hand-written SIMD math (SSE/AVX) — Phase 1 showcase.
//
// Layout choice: Array-of-Structs (AoS). Each Vec4/Quat is one __m128 holding
// {x,y,z,w} in lanes 0..3; a Mat4 is four __m128 rows. This matches DirectXMath's
// register layout exactly (XMVECTOR *is* __m128 on x64), which we exploit to
// validate against it for free.
//
// Conventions (chosen to match DirectXMath so cross-checks are 1:1):
//   * row-major matrices, row-vector transforms: v' = v * M.
//   * quaternions stored (x,y,z,w), Hamilton product.
//
// Everything is inline so it can fold into call sites. Types are 16-byte aligned
// so they live in SSE registers and in aligned containers.
// ---------------------------------------------------------------------------
#include <immintrin.h>
#include <cmath>

namespace SGE::Math {

// ============================== Vec4 (SSE) =================================
struct alignas(16) Vec4 {
    __m128 v;

    Vec4()                          : v(_mm_setzero_ps()) {}
    explicit Vec4(__m128 m)         : v(m) {}
    // _mm_set_ps takes args high->low lane, so (w,z,y,x) puts x in lane 0.
    Vec4(float x, float y, float z, float w = 0.0f) : v(_mm_set_ps(w, z, y, x)) {}

    float x() const { return _mm_cvtss_f32(v); }
    float y() const { return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(1,1,1,1))); }
    float z() const { return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(2,2,2,2))); }
    float w() const { return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(3,3,3,3))); }

    void Store(float* p) const { _mm_storeu_ps(p, v); }
};

inline Vec4 operator+(Vec4 a, Vec4 b) { return Vec4(_mm_add_ps(a.v, b.v)); }
inline Vec4 operator-(Vec4 a, Vec4 b) { return Vec4(_mm_sub_ps(a.v, b.v)); }
inline Vec4 operator*(Vec4 a, Vec4 b) { return Vec4(_mm_mul_ps(a.v, b.v)); }
inline Vec4 operator*(Vec4 a, float s){ return Vec4(_mm_mul_ps(a.v, _mm_set1_ps(s))); }
inline Vec4 operator*(float s, Vec4 a){ return a * s; }
inline Vec4 operator/(Vec4 a, float s){ return Vec4(_mm_div_ps(a.v, _mm_set1_ps(s))); }

// Unary minus / Abs flip or clear the IEEE sign bit — no arithmetic involved.
inline Vec4 operator-(Vec4 a) {
    return Vec4(_mm_xor_ps(a.v, _mm_set1_ps(-0.0f)));
}
inline Vec4 Abs(Vec4 a) {
    return Vec4(_mm_andnot_ps(_mm_set1_ps(-0.0f), a.v));
}

inline Vec4& operator+=(Vec4& a, Vec4 b) { a.v = _mm_add_ps(a.v, b.v); return a; }
inline Vec4& operator-=(Vec4& a, Vec4 b) { a.v = _mm_sub_ps(a.v, b.v); return a; }

inline Vec4 Min(Vec4 a, Vec4 b) { return Vec4(_mm_min_ps(a.v, b.v)); }
inline Vec4 Max(Vec4 a, Vec4 b) { return Vec4(_mm_max_ps(a.v, b.v)); }

// _mm_dp_ps (SSE4.1): the high nibble masks which lanes to multiply+sum, the low
// nibble masks which output lanes receive the scalar result.
inline float Dot(Vec4 a, Vec4 b)  { return _mm_cvtss_f32(_mm_dp_ps(a.v, b.v, 0xF1)); } // xyzw
inline float Dot3(Vec4 a, Vec4 b) { return _mm_cvtss_f32(_mm_dp_ps(a.v, b.v, 0x71)); } // xyz

inline float Length(Vec4 a)  { return _mm_cvtss_f32(_mm_sqrt_ss(_mm_dp_ps(a.v, a.v, 0xF1))); }
inline float Length3(Vec4 a) { return _mm_cvtss_f32(_mm_sqrt_ss(_mm_dp_ps(a.v, a.v, 0x71))); }

inline float LengthSq(Vec4 a)  { return Dot(a, a);  }
inline float LengthSq3(Vec4 a) { return Dot3(a, a); }

// Precise normalize (divide by length — not the rsqrt approximation, so results
// match the scalar/DirectXMath reference within float epsilon).
inline Vec4 Normalize(Vec4 a) {
    __m128 d = _mm_dp_ps(a.v, a.v, 0xFF);        // |a|^2 broadcast to all 4 lanes
    return Vec4(_mm_div_ps(a.v, _mm_sqrt_ps(d)));
}
inline Vec4 Normalize3(Vec4 a) {
    __m128 d = _mm_dp_ps(a.v, a.v, 0x7F);        // sum of xyz, broadcast to all lanes
    return Vec4(_mm_div_ps(a.v, _mm_sqrt_ps(d)));
}

// 3D cross product (w = 0). The classic two-shuffle form: one fewer shuffle than
// the naive four-shuffle version.
inline Vec4 Cross(Vec4 a, Vec4 b) {
    const int yzx = _MM_SHUFFLE(3, 0, 2, 1);     // (x,y,z,w) -> (y,z,x,w)
    __m128 tmp = _mm_sub_ps(
        _mm_mul_ps(a.v, _mm_shuffle_ps(b.v, b.v, yzx)),
        _mm_mul_ps(_mm_shuffle_ps(a.v, a.v, yzx), b.v));
    return Vec4(_mm_shuffle_ps(tmp, tmp, yzx));
}

// ============================== Mat4 (SSE) =================================
// Row-major: r[0..3] are the four rows. Transforms are row-vector (v * M).
struct alignas(16) Mat4 {
    __m128 r[4];

    static Mat4 Identity() {
        Mat4 m;
        m.r[0] = _mm_set_ps(0, 0, 0, 1);
        m.r[1] = _mm_set_ps(0, 0, 1, 0);
        m.r[2] = _mm_set_ps(0, 1, 0, 0);
        m.r[3] = _mm_set_ps(1, 0, 0, 0);
        return m;
    }
    static Mat4 Translation(float x, float y, float z) {
        Mat4 m = Identity();
        m.r[3] = _mm_set_ps(1, z, y, x); // translation in row 3 (row-vector convention)
        return m;
    }
    static Mat4 Scale(float x, float y, float z) {
        Mat4 m;
        m.r[0] = _mm_set_ps(0, 0, 0, x);
        m.r[1] = _mm_set_ps(0, 0, y, 0);
        m.r[2] = _mm_set_ps(0, z, 0, 0);
        m.r[3] = _mm_set_ps(1, 0, 0, 0);
        return m;
    }
};

// C = A * B (row-major). Result row i = A[i].x*B.r0 + A[i].y*B.r1 + ... — each
// scalar of A's row is broadcast across a B row and accumulated. 16 mul + 12 add
// of __m128 instead of 64 scalar mul + 48 add.
inline Mat4 Mul(const Mat4& A, const Mat4& B) {
    Mat4 out;
    for (int i = 0; i < 4; ++i) {
        __m128 ai = A.r[i];
        __m128 x = _mm_shuffle_ps(ai, ai, _MM_SHUFFLE(0,0,0,0));
        __m128 y = _mm_shuffle_ps(ai, ai, _MM_SHUFFLE(1,1,1,1));
        __m128 z = _mm_shuffle_ps(ai, ai, _MM_SHUFFLE(2,2,2,2));
        __m128 w = _mm_shuffle_ps(ai, ai, _MM_SHUFFLE(3,3,3,3));
        out.r[i] = _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(x, B.r[0]), _mm_mul_ps(y, B.r[1])),
            _mm_add_ps(_mm_mul_ps(z, B.r[2]), _mm_mul_ps(w, B.r[3])));
    }
    return out;
}

// v * M (row vector). Same broadcast-and-accumulate over the rows.
inline Vec4 Transform(Vec4 v, const Mat4& M) {
    __m128 x = _mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(0,0,0,0));
    __m128 y = _mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(1,1,1,1));
    __m128 z = _mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(2,2,2,2));
    __m128 w = _mm_shuffle_ps(v.v, v.v, _MM_SHUFFLE(3,3,3,3));
    return Vec4(_mm_add_ps(
        _mm_add_ps(_mm_mul_ps(x, M.r[0]), _mm_mul_ps(y, M.r[1])),
        _mm_add_ps(_mm_mul_ps(z, M.r[2]), _mm_mul_ps(w, M.r[3]))));
}

inline Mat4 Transpose(const Mat4& m) {
    Mat4 t = m;
    _MM_TRANSPOSE4_PS(t.r[0], t.r[1], t.r[2], t.r[3]); // intrinsic 4x4 transpose
    return t;
}

// Inverse of the upper 3x3 block; w row/column forced to identity. This is all
// the physics solver ever inverts (3x3 effective-mass blocks and inertia
// tensors are 3x3 by construction), so there is no general 4x4 inverse.
//
// Cross-product form of the adjugate: for rows r0,r1,r2 the cofactor rows are
// r1xr2, r2xr0, r0xr1 and det = r0.(r1xr2); the inverse is the TRANSPOSE of the
// cofactors over det. Caller guarantees non-singularity (effective-mass K
// matrices are SPD; inertia tensors are positive-definite).
inline Mat4 Inverse3x3(const Mat4& m) {
    Vec4 r0(m.r[0]), r1(m.r[1]), r2(m.r[2]);
    Vec4 c0 = Cross(r1, r2);
    Vec4 c1 = Cross(r2, r0);
    Vec4 c2 = Cross(r0, r1);
    float invDet = 1.0f / Dot3(r0, c0);

    Mat4 out;
    out.r[0] = c0.v;
    out.r[1] = c1.v;
    out.r[2] = c2.v;
    out.r[3] = _mm_set_ps(1, 0, 0, 0);
    out = Transpose(out);            // cofactor rows -> adjugate (w lanes are 0 -> row 3 = 0,0,0,1)
    __m128 s = _mm_set1_ps(invDet);
    out.r[0] = _mm_mul_ps(out.r[0], s);
    out.r[1] = _mm_mul_ps(out.r[1], s);
    out.r[2] = _mm_mul_ps(out.r[2], s);
    return out;
}

// ============================== Quat (SSE) =================================
// Stored (x,y,z,w). Identity is (0,0,0,1).
struct alignas(16) Quat {
    __m128 v;

    Quat()                  : v(_mm_set_ps(1, 0, 0, 0)) {}
    explicit Quat(__m128 m) : v(m) {}
    Quat(float x, float y, float z, float w) : v(_mm_set_ps(w, z, y, x)) {}

    static Quat FromAxisAngle(Vec4 axis, float angle) {
        float h = angle * 0.5f;
        float s = std::sin(h);
        Vec4 a = Normalize3(axis);
        return Quat(a.x() * s, a.y() * s, a.z() * s, std::cos(h));
    }
};

// Hamilton product a*b, fully in SSE. Each of a's components multiplies a
// sign-flipped shuffle of b; the four products are summed. (Signs/shuffles
// verified against the scalar reference in MathBenchmark.)
inline Quat Mul(Quat qa, Quat qb) {
    __m128 a = qa.v, b = qb.v;
    __m128 ax = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0,0,0,0));
    __m128 ay = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1,1,1,1));
    __m128 az = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2,2,2,2));
    __m128 aw = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3,3,3,3));

    // Sign masks (lane0..3) applied by XOR to flip the sign bit.
    const __m128 sx = _mm_castsi128_ps(_mm_set_epi32(0x80000000, 0, 0x80000000, 0)); // (+,-,+,-)
    const __m128 sy = _mm_castsi128_ps(_mm_set_epi32(0x80000000, 0x80000000, 0, 0)); // (+,+,-,-)
    const __m128 sz = _mm_castsi128_ps(_mm_set_epi32(0x80000000, 0, 0, 0x80000000)); // (-,+,+,-)

    __m128 tw = _mm_mul_ps(aw, b);
    __m128 tx = _mm_mul_ps(ax, _mm_xor_ps(_mm_shuffle_ps(b, b, _MM_SHUFFLE(0,1,2,3)), sx));
    __m128 ty = _mm_mul_ps(ay, _mm_xor_ps(_mm_shuffle_ps(b, b, _MM_SHUFFLE(1,0,3,2)), sy));
    __m128 tz = _mm_mul_ps(az, _mm_xor_ps(_mm_shuffle_ps(b, b, _MM_SHUFFLE(2,3,0,1)), sz));
    return Quat(_mm_add_ps(_mm_add_ps(tw, tx), _mm_add_ps(ty, tz)));
}

inline Quat Normalize(Quat q) {
    __m128 d = _mm_dp_ps(q.v, q.v, 0xFF);
    return Quat(_mm_div_ps(q.v, _mm_sqrt_ps(d)));
}

// Conjugate negates the vector part (for a unit quat this is the inverse).
inline Quat Conjugate(Quat q) {
    const __m128 flipXYZ = _mm_castsi128_ps(_mm_set_epi32(0, 0x80000000, 0x80000000, 0x80000000));
    return Quat(_mm_xor_ps(q.v, flipXYZ));
}

// Component-wise add/scale. NOT rotations by themselves — these exist for the
// angular-velocity integration step q' = Normalize(q + Mul(wQuat, q) * (h/2)),
// which walks q off the unit sphere and renormalizes.
inline Quat operator+(Quat a, Quat b) { return Quat(_mm_add_ps(a.v, b.v)); }
inline Quat operator*(Quat a, float s){ return Quat(_mm_mul_ps(a.v, _mm_set1_ps(s))); }

// Rotate vector v by unit quaternion q without forming the matrix:
// t = 2(u x v), v' = v + w t + u x t  (u = q.xyz). Two crosses instead of the
// naive q*v*conj(q) (two Hamilton products). Preserves v.w (Cross zeroes w).
inline Vec4 Rotate(Quat q, Vec4 v) {
    Vec4 u(q.v);
    Vec4 t = Cross(u, v) * 2.0f;
    return v + t * u.w() + Cross(u, t);
}

// Unit quaternion -> rotation matrix (row-major, row-vector; matches
// XMMatrixRotationQuaternion).
inline Mat4 ToMatrix(Quat q) {
    float x = q.v.m128_f32[0], y = q.v.m128_f32[1], z = q.v.m128_f32[2], w = q.v.m128_f32[3];
    float xx = x*x, yy = y*y, zz = z*z;
    float xy = x*y, xz = x*z, yz = y*z;
    float wx = w*x, wy = w*y, wz = w*z;
    Mat4 m;
    m.r[0] = _mm_set_ps(0.0f, 2*(xz - wy), 2*(xy + wz), 1 - 2*(yy + zz));
    m.r[1] = _mm_set_ps(0.0f, 2*(yz + wx), 1 - 2*(xx + zz), 2*(xy - wz));
    m.r[2] = _mm_set_ps(0.0f, 1 - 2*(xx + yy), 2*(yz - wx), 2*(xz + wy));
    m.r[3] = _mm_set_ps(1.0f, 0.0f, 0.0f, 0.0f);
    return m;
}

} // namespace SGE::Math
