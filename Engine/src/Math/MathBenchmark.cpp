#include "Math/MathBenchmark.h"
#include "Math/SimdMath.h"
#include "Math/ScalarMath.h"

#include <DirectXMath.h>
#include <chrono>
#include <random>
#include <cmath>
#include <algorithm>
#include <vector>
#include <intrin.h> // __cpuid

namespace SGE::Math {
namespace {

using clock_t_ = std::chrono::high_resolution_clock;

// Keep the optimizer from deleting benchmark work whose result is unused.
volatile float g_sink = 0.0f;

bool IsAVXSupported() {
    int info[4];
    __cpuid(info, 1);
    const bool osxsave = (info[2] & (1 << 27)) != 0;
    const bool avx     = (info[2] & (1 << 28)) != 0;
    if (!(osxsave && avx)) return false;
    // OS must also have enabled saving YMM state (XCR0 bits 1 and 2).
    const unsigned long long xcr0 = _xgetbv(0);
    return (xcr0 & 0x6) == 0x6;
}

template <class F>
double TimeMs(F&& f, int reps) {
    double best = 1e30;
    for (int r = 0; r < reps; ++r) {
        auto t0 = clock_t_::now();
        f();
        auto t1 = clock_t_::now();
        best = std::min(best, std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    return best;
}

// --- conversions between the three representations ---
Scalar::Mat4 ToScalar(const Mat4& m) {
    Scalar::Mat4 s;
    alignas(16) float f[4];
    for (int i = 0; i < 4; ++i) {
        _mm_store_ps(f, m.r[i]);
        s.m[i][0] = f[0]; s.m[i][1] = f[1]; s.m[i][2] = f[2]; s.m[i][3] = f[3];
    }
    return s;
}
DirectX::XMMATRIX ToXM(const Mat4& m) {
    DirectX::XMMATRIX x;
    x.r[0] = m.r[0]; x.r[1] = m.r[1]; x.r[2] = m.r[2]; x.r[3] = m.r[3];
    return x;
}
void Store16(const Mat4& m, float* out) {
    for (int i = 0; i < 4; ++i) _mm_storeu_ps(out + i * 4, m.r[i]);
}
void Store16(const DirectX::XMMATRIX& m, float* out) {
    for (int i = 0; i < 4; ++i) _mm_storeu_ps(out + i * 4, m.r[i]);
}
void Store16(const Scalar::Mat4& m, float* out) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) out[i * 4 + j] = m.m[i][j];
}
float MaxDiff(const float* a, const float* b, int n) {
    float m = 0.0f;
    for (int i = 0; i < n; ++i) m = std::max(m, std::fabs(a[i] - b[i]));
    return m;
}

Mat4 RandMat(std::mt19937& rng) {
    std::uniform_real_distribution<float> d(-1.0f, 1.0f);
    Mat4 m;
    for (int i = 0; i < 4; ++i) m.r[i] = _mm_set_ps(d(rng), d(rng), d(rng), d(rng));
    return m;
}
Vec4 RandVec(std::mt19937& rng) {
    std::uniform_real_distribution<float> d(-1.0f, 1.0f);
    return Vec4(d(rng), d(rng), d(rng), d(rng));
}

constexpr float kEps = 1e-4f;

} // namespace

BenchResults Run(bool includeTiming) {
    using namespace DirectX;
    BenchResults res;
    res.avxUsed = IsAVXSupported();
    std::mt19937 rng(20260628u);

    auto record = [&](const char* name, float err) {
        res.correctness.push_back({ name, err < kEps, err });
    };

    // ---- Correctness: SIMD vs scalar AND vs DirectXMath, over many samples ----
    {
        float err = 0.0f;
        for (int s = 0; s < 64; ++s) {
            Vec4 a = RandVec(rng), b = RandVec(rng);
            Vec4 simd = Cross(a, b);
            float fs[4]; simd.Store(fs);
            Scalar::Vec4 sa{ a.x(), a.y(), a.z(), 0 }, sb{ b.x(), b.y(), b.z(), 0 };
            Scalar::Vec4 sc = Scalar::Cross3(sa, sb);
            err = std::max(err, std::max({ std::fabs(fs[0]-sc.x), std::fabs(fs[1]-sc.y), std::fabs(fs[2]-sc.z) }));
            float fd[4]; _mm_storeu_ps(fd, XMVector3Cross(a.v, b.v));
            err = std::max(err, MaxDiff(fs, fd, 3));
        }
        record("Vec3 Cross", err);
    }
    {
        float err = 0.0f;
        for (int s = 0; s < 64; ++s) {
            Vec4 a = RandVec(rng);
            float fs[4]; Normalize(a).Store(fs);
            Scalar::Vec4 sn = Scalar::Normalize({ a.x(), a.y(), a.z(), a.w() });
            err = std::max(err, std::max({ std::fabs(fs[0]-sn.x), std::fabs(fs[1]-sn.y), std::fabs(fs[2]-sn.z), std::fabs(fs[3]-sn.w) }));
            float fd[4]; _mm_storeu_ps(fd, XMVector4Normalize(a.v));
            err = std::max(err, MaxDiff(fs, fd, 4));
        }
        record("Vec4 Normalize", err);
    }
    {
        float err = 0.0f;
        for (int s = 0; s < 32; ++s) {
            Mat4 A = RandMat(rng), B = RandMat(rng);
            float fs[16], fc[16], fd[16];
            Store16(Mul(A, B), fs);
            Store16(Scalar::Mul(ToScalar(A), ToScalar(B)), fc);
            Store16(XMMatrixMultiply(ToXM(A), ToXM(B)), fd);
            err = std::max(err, std::max(MaxDiff(fs, fc, 16), MaxDiff(fs, fd, 16)));
        }
        record("Mat4 Multiply", err);
    }
    {
        float err = 0.0f;
        for (int s = 0; s < 64; ++s) {
            Mat4 M = RandMat(rng);
            Vec4 v = RandVec(rng);
            float fs[4]; Transform(v, M).Store(fs);
            Scalar::Vec4 sv = Scalar::Transform({ v.x(), v.y(), v.z(), v.w() }, ToScalar(M));
            err = std::max(err, std::max({ std::fabs(fs[0]-sv.x), std::fabs(fs[1]-sv.y), std::fabs(fs[2]-sv.z), std::fabs(fs[3]-sv.w) }));
            float fd[4]; _mm_storeu_ps(fd, XMVector4Transform(v.v, ToXM(M)));
            err = std::max(err, MaxDiff(fs, fd, 4));
        }
        record("Vec4 Transform", err);
    }
    {
        float err = 0.0f;
        for (int s = 0; s < 64; ++s) {
            Vec4 ra = RandVec(rng), rb = RandVec(rng);
            Quat qa = Normalize(Quat(ra.v)), qb = Normalize(Quat(rb.v));
            float fs[4]; Quat qm = Mul(qa, qb); _mm_storeu_ps(fs, qm.v);
            Scalar::Vec4 sa{ qa.v.m128_f32[0], qa.v.m128_f32[1], qa.v.m128_f32[2], qa.v.m128_f32[3] };
            Scalar::Vec4 sb{ qb.v.m128_f32[0], qb.v.m128_f32[1], qb.v.m128_f32[2], qb.v.m128_f32[3] };
            Scalar::Vec4 sc = Scalar::QuatMul(sa, sb);
            err = std::max(err, std::max({ std::fabs(fs[0]-sc.x), std::fabs(fs[1]-sc.y), std::fabs(fs[2]-sc.z), std::fabs(fs[3]-sc.w) }));
            // DirectXMath multiplies "rotation Q1 then Q2" = Q2*Q1, so pass (qb, qa) to get qa*qb.
            float fd[4]; _mm_storeu_ps(fd, XMQuaternionMultiply(qb.v, qa.v));
            err = std::max(err, MaxDiff(fs, fd, 4));
        }
        record("Quat Multiply", err);
    }
    {
        float err = 0.0f;
        for (int s = 0; s < 32; ++s) {
            Vec4 r = RandVec(rng);
            Quat q = Normalize(Quat(r.v));
            float fs[16], fd[16];
            Store16(ToMatrix(q), fs);
            Store16(XMMatrixRotationQuaternion(q.v), fd);
            err = std::max(err, MaxDiff(fs, fd, 16));
        }
        record("Quat -> Matrix", err);
    }

    // ---- physics-support ops (added for the rigid-body solver) ----
    {
        float err = 0.0f;
        for (int s = 0; s < 64; ++s) {
            Vec4 a = RandVec(rng), b = RandVec(rng);
            Scalar::Vec4 sa{ a.x(), a.y(), a.z(), a.w() }, sb{ b.x(), b.y(), b.z(), b.w() };
            float fs[4], fc[4], fd[4];

            Min(a, b).Store(fs);
            Scalar::Vec4 sc = Scalar::Min(sa, sb);
            fc[0]=sc.x; fc[1]=sc.y; fc[2]=sc.z; fc[3]=sc.w;
            _mm_storeu_ps(fd, XMVectorMin(a.v, b.v));
            err = std::max(err, std::max(MaxDiff(fs, fc, 4), MaxDiff(fs, fd, 4)));

            Max(a, b).Store(fs);
            sc = Scalar::Max(sa, sb);
            fc[0]=sc.x; fc[1]=sc.y; fc[2]=sc.z; fc[3]=sc.w;
            _mm_storeu_ps(fd, XMVectorMax(a.v, b.v));
            err = std::max(err, std::max(MaxDiff(fs, fc, 4), MaxDiff(fs, fd, 4)));

            Abs(a).Store(fs);
            sc = Scalar::Abs(sa);
            fc[0]=sc.x; fc[1]=sc.y; fc[2]=sc.z; fc[3]=sc.w;
            _mm_storeu_ps(fd, XMVectorAbs(a.v));
            err = std::max(err, std::max(MaxDiff(fs, fc, 4), MaxDiff(fs, fd, 4)));

            (-a).Store(fs);
            sc = Scalar::Neg(sa);
            fc[0]=sc.x; fc[1]=sc.y; fc[2]=sc.z; fc[3]=sc.w;
            _mm_storeu_ps(fd, XMVectorNegate(a.v));
            err = std::max(err, std::max(MaxDiff(fs, fc, 4), MaxDiff(fs, fd, 4)));

            err = std::max(err, std::fabs(LengthSq(a)  - Scalar::LengthSq(sa)));
            err = std::max(err, std::fabs(LengthSq3(a) - Scalar::LengthSq3(sa)));
            err = std::max(err, std::fabs(LengthSq(a)  - XMVectorGetX(XMVector4LengthSq(a.v))));
            err = std::max(err, std::fabs(LengthSq3(a) - XMVectorGetX(XMVector3LengthSq(a.v))));
        }
        record("Vec4 Min/Max/Abs/Neg/LenSq", err);
    }
    {
        float err = 0.0f;
        for (int s = 0; s < 64; ++s) {
            Quat q = Normalize(Quat(RandVec(rng).v));
            float fs[4], fd[4];
            _mm_storeu_ps(fs, Conjugate(q).v);
            Scalar::Vec4 sc = Scalar::QuatConjugate({ q.v.m128_f32[0], q.v.m128_f32[1], q.v.m128_f32[2], q.v.m128_f32[3] });
            err = std::max(err, std::max({ std::fabs(fs[0]-sc.x), std::fabs(fs[1]-sc.y), std::fabs(fs[2]-sc.z), std::fabs(fs[3]-sc.w) }));
            _mm_storeu_ps(fd, XMQuaternionConjugate(q.v));
            err = std::max(err, MaxDiff(fs, fd, 4));
        }
        record("Quat Conjugate", err);
    }
    {
        // Rotate(q,v) vs scalar, vs XMVector3Rotate, AND vs the matrix path
        // Transform(v, ToMatrix(q)) — the identity the solver's inertia-transform
        // convention rests on.
        float err = 0.0f;
        for (int s = 0; s < 64; ++s) {
            Quat q = Normalize(Quat(RandVec(rng).v));
            Vec4 r = RandVec(rng);
            Vec4 v(r.x(), r.y(), r.z(), 0.0f);
            float fs[4], fc[4], fd[4];
            Rotate(q, v).Store(fs);

            Scalar::Vec4 sq{ q.v.m128_f32[0], q.v.m128_f32[1], q.v.m128_f32[2], q.v.m128_f32[3] };
            Scalar::Vec4 sc = Scalar::QuatRotate(sq, { v.x(), v.y(), v.z(), 0.0f });
            fc[0]=sc.x; fc[1]=sc.y; fc[2]=sc.z; fc[3]=sc.w;
            err = std::max(err, MaxDiff(fs, fc, 4));

            _mm_storeu_ps(fd, XMVector3Rotate(v.v, q.v));
            err = std::max(err, MaxDiff(fs, fd, 3));

            float fm[4]; Transform(v, ToMatrix(q)).Store(fm);
            err = std::max(err, MaxDiff(fs, fm, 4));
        }
        record("Quat Rotate", err);
    }
    {
        // Well-conditioned inputs: rotation (det 1) times a diagonal scale in
        // [0.5, 2] — random matrices can be near-singular and would blow the
        // epsilon on any implementation.
        float err = 0.0f;
        std::uniform_real_distribution<float> sd(0.5f, 2.0f);
        for (int s = 0; s < 32; ++s) {
            Mat4 M = ToMatrix(Normalize(Quat(RandVec(rng).v)));
            float sx = sd(rng), sy = sd(rng), sz = sd(rng);
            M.r[0] = _mm_mul_ps(M.r[0], _mm_set1_ps(sx));
            M.r[1] = _mm_mul_ps(M.r[1], _mm_set1_ps(sy));
            M.r[2] = _mm_mul_ps(M.r[2], _mm_set1_ps(sz));

            float fs[16], fc[16], fd[16];
            Store16(Inverse3x3(M), fs);
            Store16(Scalar::Inverse3x3(ToScalar(M)), fc);
            Store16(XMMatrixInverse(nullptr, ToXM(M)), fd);
            err = std::max(err, std::max(MaxDiff(fs, fc, 16), MaxDiff(fs, fd, 16)));
        }
        record("Mat4 Inverse3x3", err);
    }
    {
        // One angular-velocity integration step, SIMD pipeline vs the same
        // formula composed from scalar ops: q' = Normalize(q + (wq * q) * h/2).
        float err = 0.0f;
        const float h = 1.0f / 120.0f;
        for (int s = 0; s < 64; ++s) {
            Quat q = Normalize(Quat(RandVec(rng).v));
            Vec4 w = RandVec(rng) * 3.0f; // up to ~3 rad/s per axis
            Quat wq(w.x(), w.y(), w.z(), 0.0f);
            float fs[4]; _mm_storeu_ps(fs, Normalize(q + Mul(wq, q) * (0.5f * h)).v);

            Scalar::Vec4 sq{ q.v.m128_f32[0], q.v.m128_f32[1], q.v.m128_f32[2], q.v.m128_f32[3] };
            Scalar::Vec4 swq{ w.x(), w.y(), w.z(), 0.0f };
            Scalar::Vec4 sn = Scalar::Normalize(Scalar::Add(sq, Scalar::Scale(Scalar::QuatMul(swq, sq), 0.5f * h)));
            err = std::max(err, std::max({ std::fabs(fs[0]-sn.x), std::fabs(fs[1]-sn.y), std::fabs(fs[2]-sn.z), std::fabs(fs[3]-sn.w) }));
        }
        record("Quat Integrate step", err);
    }

    res.allCorrect = true;
    for (const auto& c : res.correctness) res.allCorrect &= c.passed;

    if (!includeTiming)
        return res;

    // -------------------------- Benchmarks --------------------------
    constexpr int kReps = 12;

    // Mat4 multiply over an array.
    {
        const int N = 30000;
        std::vector<Mat4> A(N), B(N);
        for (int i = 0; i < N; ++i) { A[i] = RandMat(rng); B[i] = RandMat(rng); }
        std::vector<Scalar::Mat4> sA(N), sB(N);
        for (int i = 0; i < N; ++i) { sA[i] = ToScalar(A[i]); sB[i] = ToScalar(B[i]); }
        const auto* xA = reinterpret_cast<const DirectX::XMMATRIX*>(A.data());
        const auto* xB = reinterpret_cast<const DirectX::XMMATRIX*>(B.data());

        double simd = TimeMs([&]{ float acc=0; for (int i=0;i<N;++i){ Mat4 c=Mul(A[i],B[i]); acc+=_mm_cvtss_f32(c.r[0]); } g_sink=acc; }, kReps);
        double scal = TimeMs([&]{ float acc=0; for (int i=0;i<N;++i){ Scalar::Mat4 c=Scalar::Mul(sA[i],sB[i]); acc+=c.m[0][0]; } g_sink=acc; }, kReps);
        double dxm  = TimeMs([&]{ float acc=0; for (int i=0;i<N;++i){ DirectX::XMMATRIX c=DirectX::XMMatrixMultiply(xA[i],xB[i]); acc+=_mm_cvtss_f32(c.r[0]); } g_sink=acc; }, kReps);
        res.bench.push_back({ "Mat4 multiply (30k)", scal, simd, dxm });
    }

    // Vec4 transform by a matrix over an array.
    {
        const int N = 300000;
        std::vector<Vec4> V(N);
        for (int i = 0; i < N; ++i) V[i] = RandVec(rng);
        Mat4 M = RandMat(rng);
        Scalar::Mat4 sM = ToScalar(M);
        DirectX::XMMATRIX xM = ToXM(M);

        double simd = TimeMs([&]{ float acc=0; for (int i=0;i<N;++i){ acc+=Transform(V[i],M).x(); } g_sink=acc; }, kReps);
        double scal = TimeMs([&]{ float acc=0; for (int i=0;i<N;++i){ acc+=Scalar::Transform({V[i].x(),V[i].y(),V[i].z(),V[i].w()},sM).x; } g_sink=acc; }, kReps);
        double dxm  = TimeMs([&]{ float acc=0; for (int i=0;i<N;++i){ acc+=DirectX::XMVectorGetX(DirectX::XMVector4Transform(V[i].v,xM)); } g_sink=acc; }, kReps);
        res.bench.push_back({ "Vec4 transform (300k)", scal, simd, dxm });
    }

    // SoA Vec3 normalize — the AVX showcase (8 vectors per register).
    {
        const int N = 500000;
        std::uniform_real_distribution<float> d(-1.0f, 1.0f);
        std::vector<float> xs(N), ys(N), zs(N), xa(N), ya(N), za(N);
        for (int i = 0; i < N; ++i) { float x=d(rng)+1.1f,y=d(rng)+1.1f,z=d(rng)+1.1f; xs[i]=xa[i]=x; ys[i]=ya[i]=y; zs[i]=za[i]=z; }

        double scal = TimeMs([&]{
            for (int i=0;i<N;++i){ float l=std::sqrt(xs[i]*xs[i]+ys[i]*ys[i]+zs[i]*zs[i]); xs[i]/=l; ys[i]/=l; zs[i]/=l; }
            g_sink = xs[0];
        }, kReps);

        double avx = -1.0;
        if (res.avxUsed) {
            avx = TimeMs([&]{
                int i = 0;
                for (; i + 8 <= N; i += 8) {
                    __m256 X=_mm256_loadu_ps(&xa[i]), Y=_mm256_loadu_ps(&ya[i]), Z=_mm256_loadu_ps(&za[i]);
                    __m256 l2=_mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(X,X),_mm256_mul_ps(Y,Y)),_mm256_mul_ps(Z,Z));
                    __m256 inv=_mm256_div_ps(_mm256_set1_ps(1.0f), _mm256_sqrt_ps(l2));
                    _mm256_storeu_ps(&xa[i], _mm256_mul_ps(X,inv));
                    _mm256_storeu_ps(&ya[i], _mm256_mul_ps(Y,inv));
                    _mm256_storeu_ps(&za[i], _mm256_mul_ps(Z,inv));
                }
                for (; i < N; ++i){ float l=std::sqrt(xa[i]*xa[i]+ya[i]*ya[i]+za[i]*za[i]); xa[i]/=l; ya[i]/=l; za[i]/=l; }
                g_sink = xa[0];
            }, kReps);
        }
        // dxmath column is N/A for the SoA layout.
        res.bench.push_back({ "SoA Vec3 normalize, AVX (500k)", scal, avx, -1.0 });
    }

    return res;
}

} // namespace SGE::Math
