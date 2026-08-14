#include "Physics/Narrowphase.h"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace SGE::Physics::Narrowphase {

using namespace SGE::Math;

namespace {

// A body's local axes in world space = rows 0..2 of its rotation matrix
// (row-vector convention: local (1,0,0) maps to row 0). Rotation rows carry
// w = 0, so Dot3/Cross treat them as pure directions.
void AxesOf(const Quat& q, Vec4 u[3]) {
    Mat4 R = ToMatrix(q);
    u[0] = Vec4(R.r[0]);
    u[1] = Vec4(R.r[1]);
    u[2] = Vec4(R.r[2]);
}

Vec4 AxisUnit(int i, float sign) {
    return Vec4(i == 0 ? sign : 0.0f, i == 1 ? sign : 0.0f, i == 2 ? sign : 0.0f, 0.0f);
}

// Face contacts also keep points slightly ABOVE the surface (negative
// penetration). Without this a resting box that tilts by microns loses two of
// its four points — and their warm-started impulses — every time it rocks,
// which pumps the rocking until the stack walks apart. The solver treats such
// points as speculative: they only stop the bodies from closing the remaining
// gap within one substep (see ContactSolver::Presolve).
constexpr float kPersistTol = 0.01f;

// Candidate contact point: position, penetration depth, warm-start feature id.
struct Hit {
    Vec4     p;
    float    pen;
    uint32_t id;
};

// Deepest-first, id as the deterministic tie-break. Insertion sort — the
// candidate sets here are at most 8 entries.
void SortHits(Hit* hits, int count) {
    for (int i = 1; i < count; ++i) {
        Hit h = hits[i];
        int j = i - 1;
        for (; j >= 0 && (hits[j].pen < h.pen
                          || (hits[j].pen == h.pen && hits[j].id > h.id)); --j)
            hits[j + 1] = hits[j];
        hits[j + 1] = h;
    }
}

// Vertex carried through Sutherland-Hodgman clipping. The id survives from
// the incident-face corner (or is minted at a plane crossing) and becomes the
// FeatureId the warm-starter matches on.
struct ClipVert {
    Vec4     p;
    uint32_t id;
};

// Clip polygon `in` against half-space  Dot3(p, n) <= offset. Classic
// Sutherland-Hodgman: emit kept vertices, and a fresh vertex (with a
// deterministic id derived from the crossed edge + plane) at each crossing.
// An intersection is emitted only when the edge STRICTLY straddles the plane —
// a vertex exactly on it (the common case for axis-aligned stacks) is already
// kept by the d0 <= 0 branch, and emitting its t = 0 duplicate too would grow
// the polygon without bound. The hard cap is belt-and-braces for float noise.
constexpr int kMaxClipVerts = 8;

int ClipAgainstPlane(const ClipVert* in, int inCount, Vec4 n, float offset,
                     uint32_t planeIdx, ClipVert* out) {
    int outCount = 0;
    for (int i = 0; i < inCount; ++i) {
        const ClipVert& v0 = in[i];
        const ClipVert& v1 = in[(i + 1) % inCount];
        float d0 = Dot3(v0.p, n) - offset;
        float d1 = Dot3(v1.p, n) - offset;

        if (d0 <= 0.0f && outCount < kMaxClipVerts)
            out[outCount++] = v0;
        if (((d0 < 0.0f && d1 > 0.0f) || (d0 > 0.0f && d1 < 0.0f)) && outCount < kMaxClipVerts) {
            float tt = d0 / (d0 - d1);
            out[outCount++] = { v0.p + (v1.p - v0.p) * tt,
                                0x40u | (planeIdx << 4) | (v0.id & 0xFu) };
        }
    }
    return outCount;
}

// World-space segment (the capsule's core line, caps excluded).
void CapsuleSegment(const RigidBody& c, Vec4& p0, Vec4& p1) {
    Mat4 R = ToMatrix(c.Orientation);
    float he[4]; c.Shape.Extents.Store(he);
    const Vec4 axis = Vec4(R.r[1]);                          // local +Y in world
    p0 = c.Position - axis * he[0];
    p1 = c.Position + axis * he[0];
}

Vec4 ClosestOnSegment(Vec4 a, Vec4 b, Vec4 p, float& tOut) {
    const Vec4 ab = b - a;
    const float lenSq = LengthSq3(ab);
    tOut = lenSq > 1e-12f ? std::clamp(Dot3(p - a, ab) / lenSq, 0.0f, 1.0f) : 0.0f;
    return a + ab * tOut;
}

// Closest points between segments p1q1 and p2q2 (Ericson, RTCD 5.1.9).
void ClosestPtSegmentSegment(Vec4 p1, Vec4 q1, Vec4 p2, Vec4 q2,
                             float& s, float& t, Vec4& c1, Vec4& c2) {
    const Vec4 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const float A = LengthSq3(d1), E = LengthSq3(d2), F = Dot3(d2, r);
    if (A <= 1e-12f && E <= 1e-12f) { s = t = 0.0f; c1 = p1; c2 = p2; return; }
    if (A <= 1e-12f) {
        s = 0.0f; t = std::clamp(F / E, 0.0f, 1.0f);
    } else {
        const float C = Dot3(d1, r);
        if (E <= 1e-12f) {
            t = 0.0f; s = std::clamp(-C / A, 0.0f, 1.0f);
        } else {
            const float B = Dot3(d1, d2);
            const float denom = A * E - B * B;
            s = denom > 1e-12f ? std::clamp((B * F - C * E) / denom, 0.0f, 1.0f) : 0.0f;
            t = (B * s + F) / E;
            if (t < 0.0f)      { t = 0.0f; s = std::clamp(-C / A, 0.0f, 1.0f); }
            else if (t > 1.0f) { t = 1.0f; s = std::clamp((B - C) / A, 0.0f, 1.0f); }
        }
    }
    c1 = p1 + (q1 - p1) * s;
    c2 = p2 + (q2 - p2) * t;
}

// Sphere-vs-box core in BOX LOCAL space, shared by SphereBox and CapsuleBox.
// margin allows near-miss (speculative) hits: pen may come back as low as
// -margin. Outputs the local outward normal (box surface -> sphere center)
// and the local point on the box surface.
bool SphereVsBoxLocal(Vec4 local, float r, const float he[4], float margin,
                      Vec4& nLocal, Vec4& onBoxLocal, float& pen) {
    float lf[4]; local.Store(lf);
    float cl[3];
    bool inside = true;
    for (int k = 0; k < 3; ++k) {
        cl[k] = std::clamp(lf[k], -he[k], he[k]);
        inside &= (cl[k] == lf[k]);
    }
    if (!inside) {
        const Vec4 closest(cl[0], cl[1], cl[2], 0.0f);
        const Vec4 delta = local - closest;
        const float distSq = LengthSq3(delta);
        if (distSq > (r + margin) * (r + margin)) return false;
        const float dist = std::sqrt(std::max(distSq, 1e-12f));
        nLocal     = delta / dist;
        onBoxLocal = closest;
        pen        = r - dist;
    } else {
        int   axis = 0;
        float minDepth = FLT_MAX, sign = 1.0f;
        for (int k = 0; k < 3; ++k) {
            const float depth = he[k] - std::fabs(lf[k]);
            if (depth < minDepth) { minDepth = depth; axis = k; sign = lf[k] >= 0.0f ? 1.0f : -1.0f; }
        }
        nLocal = AxisUnit(axis, sign);
        float ff[4]; local.Store(ff);
        ff[axis]   = sign * he[axis];
        onBoxLocal = Vec4(ff[0], ff[1], ff[2], 0.0f);
        pen        = r + minDepth;
    }
    return true;
}

} // namespace

// ============================ sphere routines ==============================

bool SphereSphere(const RigidBody& a, const RigidBody& b, Manifold& m) {
    Vec4 d = b.Position - a.Position;
    const float rSum   = a.Shape.Radius + b.Shape.Radius;
    const float distSq = LengthSq3(d);
    if (distSq > rSum * rSum) return false;

    const float dist = std::sqrt(distSq);
    Vec4 n = dist > 1e-6f ? d / dist : Vec4(1, 0, 0, 0); // coincident centers: any axis

    m.Normal = n;
    ContactPoint& cp = m.Points[0];
    cp = ContactPoint{};
    // Midpoint of the two surface points along the normal.
    Vec4 surfA = a.Position + n * a.Shape.Radius;
    Vec4 surfB = b.Position - n * b.Shape.Radius;
    cp.Position    = (surfA + surfB) * 0.5f;
    cp.Penetration = rSum - dist;
    cp.FeatureId   = 0;
    m.Count = 1;
    return true;
}

bool SpherePlane(const RigidBody& sphere, const RigidBody& plane, Manifold& m) {
    const Vec4  n = plane.Shape.Extents;                    // unit normal, w = d
    float pe[4]; plane.Shape.Extents.Store(pe);
    const float d = pe[3];

    const float signedDist = Dot3(sphere.Position, n) - d;
    const float pen        = sphere.Shape.Radius - signedDist;
    if (pen < 0.0f) return false;

    m.Normal = -n;                                          // A (sphere) -> B (plane)
    ContactPoint& cp = m.Points[0];
    cp = ContactPoint{};
    cp.Position    = sphere.Position - n * signedDist;      // projected onto the plane
    cp.Penetration = pen;
    cp.FeatureId   = 0;
    m.Count = 1;
    return true;
}

bool SphereBox(const RigidBody& sphere, const RigidBody& box, Manifold& m) {
    // Sphere center in box space; the shared local-space core does the rest.
    Vec4 local = Rotate(Conjugate(box.Orientation), sphere.Position - box.Position);
    float he[4]; box.Shape.Extents.Store(he);

    Vec4 nLocal, onBox;
    float pen;
    if (!SphereVsBoxLocal(local, sphere.Shape.Radius, he, 0.0f, nLocal, onBox, pen))
        return false;

    m.Normal = -Rotate(box.Orientation, nLocal);            // A (sphere) -> B (box)
    ContactPoint& cp = m.Points[0];
    cp = ContactPoint{};
    cp.Position    = box.Position + Rotate(box.Orientation, onBox);
    cp.Penetration = pen;
    cp.FeatureId   = 0;
    m.Count = 1;
    return true;
}

// ============================ capsule routines =============================

bool SphereCapsule(const RigidBody& sphere, const RigidBody& capsule, Manifold& m) {
    // Closest point on the capsule's core segment, then sphere-vs-sphere.
    Vec4 p0, p1;
    CapsuleSegment(capsule, p0, p1);
    float t;
    const Vec4 c = ClosestOnSegment(p0, p1, sphere.Position, t);

    const Vec4  d      = c - sphere.Position;
    const float rSum   = sphere.Shape.Radius + capsule.Shape.Radius;
    const float distSq = LengthSq3(d);
    if (distSq > rSum * rSum) return false;

    const float dist = std::sqrt(std::max(distSq, 1e-12f));
    const Vec4  n    = dist > 1e-6f ? d / dist : Vec4(1, 0, 0, 0);

    m.Normal = n;                                            // A (sphere) -> B (capsule)
    ContactPoint& cp = m.Points[0];
    cp = ContactPoint{};
    cp.Position    = (sphere.Position + n * sphere.Shape.Radius + c - n * capsule.Shape.Radius) * 0.5f;
    cp.Penetration = rSum - dist;
    cp.FeatureId   = 0;
    m.Count = 1;
    return true;
}

bool CapsulePlane(const RigidBody& capsule, const RigidBody& plane, Manifold& m) {
    const Vec4 n = plane.Shape.Extents;
    float pe[4]; plane.Shape.Extents.Store(pe);
    const float d = pe[3];
    const float r = capsule.Shape.Radius;

    // Both cap centers vs the plane: a lying capsule rests on TWO contacts
    // (endpoint index = FeatureId), a leaning one on the deeper end. The
    // persistence margin keeps the barely-lifted end warm, like BoxPlane.
    Vec4 ends[2];
    CapsuleSegment(capsule, ends[0], ends[1]);

    Hit hits[2];
    int hitCount = 0;
    bool anyTouching = false;
    for (uint32_t i = 0; i < 2; ++i) {
        const float signedDist = Dot3(ends[i], n) - d;
        const float pen        = r - signedDist;
        if (pen > -kPersistTol) {
            hits[hitCount++] = { ends[i] - n * signedDist, pen, i };
            anyTouching |= pen > 0.0f;
        }
    }
    if (hitCount == 0 || !anyTouching) return false;

    m.Normal = -n;                                           // A (capsule) -> B (plane)
    m.Count  = uint32_t(hitCount);
    for (int i = 0; i < hitCount; ++i) {
        ContactPoint& cp = m.Points[i];
        cp = ContactPoint{};
        cp.Position    = hits[i].p;
        cp.Penetration = hits[i].pen;
        cp.FeatureId   = hits[i].id;
    }
    return true;
}

bool CapsuleCapsule(const RigidBody& a, const RigidBody& b, Manifold& m) {
    Vec4 a0, a1, b0, b1;
    CapsuleSegment(a, a0, a1);
    CapsuleSegment(b, b0, b1);
    const float rSum = a.Shape.Radius + b.Shape.Radius;

    const Vec4  dA = a1 - a0, dB = b1 - b0;
    const float lA = Length3(dA), lB = Length3(dB);

    // Nearly parallel with axis overlap: emit TWO contacts (the ends of the
    // overlap window) so a capsule lying on another rests instead of rocking.
    if (lA > 1e-6f && lB > 1e-6f && std::fabs(Dot3(dA, dB)) > 0.999f * lA * lB) {
        auto tOnA = [&](Vec4 p) { return Dot3(p - a0, dA) / (lA * lA); };
        const float u0 = std::clamp(std::min(tOnA(b0), tOnA(b1)), 0.0f, 1.0f);
        const float u1 = std::clamp(std::max(tOnA(b0), tOnA(b1)), 0.0f, 1.0f);
        if (u1 - u0 > 1e-3f) {
            Hit hits[2];
            Vec4 normals[2];
            int hitCount = 0;
            bool anyTouching = false;
            const float ts[2] = { u0, u1 };
            for (uint32_t i = 0; i < 2; ++i) {
                const Vec4 pa = a0 + dA * ts[i];
                float s;
                const Vec4 pb = ClosestOnSegment(b0, b1, pa, s);
                const Vec4 d  = pb - pa;
                const float dist = Length3(d);
                const float pen  = rSum - dist;
                if (pen > -kPersistTol) {
                    const Vec4 n = dist > 1e-6f ? d / dist : Vec4(1, 0, 0, 0);
                    normals[hitCount] = n;
                    hits[hitCount++]  = { (pa + n * a.Shape.Radius + pb - n * b.Shape.Radius) * 0.5f,
                                          pen, i };
                    anyTouching |= pen > 0.0f;
                }
            }
            if (hitCount > 0 && anyTouching) {
                SortHits(hits, hitCount);
                m.Normal = normals[0];                       // parallel: both agree
                m.Count  = uint32_t(hitCount);
                for (int i = 0; i < hitCount; ++i) {
                    ContactPoint& cp = m.Points[i];
                    cp = ContactPoint{};
                    cp.Position    = hits[i].p;
                    cp.Penetration = hits[i].pen;
                    cp.FeatureId   = hits[i].id;
                }
                return true;
            }
            return false;
        }
    }

    // General case: closest points of the two segments, sphere-vs-sphere there.
    float s, t;
    Vec4 c1, c2;
    ClosestPtSegmentSegment(a0, a1, b0, b1, s, t, c1, c2);
    const Vec4  d      = c2 - c1;
    const float distSq = LengthSq3(d);
    if (distSq > rSum * rSum) return false;

    const float dist = std::sqrt(std::max(distSq, 1e-12f));
    const Vec4  n    = dist > 1e-6f ? d / dist : Vec4(1, 0, 0, 0);

    m.Normal = n;
    ContactPoint& cp = m.Points[0];
    cp = ContactPoint{};
    cp.Position    = (c1 + n * a.Shape.Radius + c2 - n * b.Shape.Radius) * 0.5f;
    cp.Penetration = rSum - dist;
    cp.FeatureId   = 2;
    m.Count = 1;
    return true;
}

bool CapsuleBox(const RigidBody& capsule, const RigidBody& box, Manifold& m) {
    const float r = capsule.Shape.Radius;
    float he[4]; box.Shape.Extents.Store(he);

    // Segment in box-local space.
    Vec4 w0, w1;
    CapsuleSegment(capsule, w0, w1);
    const Quat invQ = Conjugate(box.Orientation);
    const Vec4 l0 = Rotate(invQ, w0 - box.Position);
    const Vec4 l1 = Rotate(invQ, w1 - box.Position);

    // Distance from a segment point to the SOLID box is convex in t, so a
    // fixed-count ternary search finds the closest parameter deterministically
    // (fixed iterations keep the serial/parallel trajectories bit-identical).
    auto distSqAt = [&](float t) {
        float p[4]; (l0 + (l1 - l0) * t).Store(p);
        float dsq = 0.0f;
        for (int k = 0; k < 3; ++k) {
            const float excess = std::max(std::fabs(p[k]) - he[k], 0.0f);
            dsq += excess * excess;
        }
        return dsq;
    };
    float lo = 0.0f, hi = 1.0f;
    for (int it = 0; it < 40; ++it) {
        const float m1 = lo + (hi - lo) / 3.0f;
        const float m2 = hi - (hi - lo) / 3.0f;
        if (distSqAt(m1) <= distSqAt(m2)) hi = m2; else lo = m1;
    }
    const float tStar = 0.5f * (lo + hi);

    // Candidates: the closest interior point plus both cap centers — the caps
    // give a lying capsule its second contact. Skip tStar when it degenerates
    // onto an endpoint.
    struct Cand { float t; uint32_t id; };
    Cand cands[3];
    int  candCount = 0;
    if (tStar > 0.02f && tStar < 0.98f) cands[candCount++] = { tStar, 0 };
    cands[candCount++] = { 0.0f, 1 };
    cands[candCount++] = { 1.0f, 2 };

    Hit  hits[3];
    Vec4 normalsLocal[3];
    int  hitCount = 0;
    bool anyTouching = false;
    for (int c = 0; c < candCount; ++c) {
        const Vec4 pt = l0 + (l1 - l0) * cands[c].t;
        Vec4 nLocal, onBox;
        float pen;
        if (!SphereVsBoxLocal(pt, r, he, kPersistTol, nLocal, onBox, pen)) continue;
        normalsLocal[hitCount] = nLocal;
        hits[hitCount++] = { onBox, pen, cands[c].id };
        anyTouching |= pen > 0.0f;
    }
    if (hitCount == 0 || !anyTouching) return false;

    // Deepest first; its normal represents the manifold (identical for a
    // face-resting capsule, best available in edge cases).
    int deepest = 0;
    for (int i = 1; i < hitCount; ++i)
        if (hits[i].pen > hits[deepest].pen ||
            (hits[i].pen == hits[deepest].pen && hits[i].id < hits[deepest].id))
            deepest = i;

    m.Normal = -Rotate(box.Orientation, normalsLocal[deepest]);  // A (capsule) -> B (box)
    SortHits(hits, hitCount);
    m.Count = uint32_t(std::min(hitCount, 2));
    for (uint32_t i = 0; i < m.Count; ++i) {
        ContactPoint& cp = m.Points[i];
        cp = ContactPoint{};
        cp.Position    = box.Position + Rotate(box.Orientation, hits[i].p);
        cp.Penetration = hits[i].pen;
        cp.FeatureId   = hits[i].id;
    }
    return true;
}

// ============================= box routines ================================

bool BoxPlane(const RigidBody& box, const RigidBody& plane, Manifold& m) {
    const Vec4 n = plane.Shape.Extents;
    float pe[4]; plane.Shape.Extents.Store(pe);
    const float d = pe[3];

    Vec4 u[3];
    AxesOf(box.Orientation, u);
    float he[4]; box.Shape.Extents.Store(he);

    // Test all 8 corners; corner index bits select the sign per axis, and that
    // index IS the feature id (stable while the box rests on the same corners).
    Hit hits[8];
    int hitCount = 0;
    bool anyTouching = false;
    for (uint32_t idx = 0; idx < 8; ++idx) {
        Vec4 corner = box.Position;
        for (int k = 0; k < 3; ++k)
            corner += u[k] * (((idx >> k) & 1) ? he[k] : -he[k]);
        const float signedDist = Dot3(corner, n) - d;
        if (signedDist < kPersistTol) {
            hits[hitCount++] = { corner, -signedDist, idx };
            anyTouching |= signedDist < 0.0f;
        }
    }
    if (hitCount == 0 || !anyTouching) return false;

    // Deepest 4 win (a resting box touches with exactly 4).
    SortHits(hits, hitCount);

    m.Normal = -n;                                          // A (box) -> B (plane)
    m.Count  = std::min(hitCount, 4);
    for (uint32_t i = 0; i < m.Count; ++i) {
        ContactPoint& cp = m.Points[i];
        cp = ContactPoint{};
        cp.Position    = hits[i].p;
        cp.Penetration = hits[i].pen;
        cp.FeatureId   = hits[i].id;
    }
    return true;
}

bool BoxBox(const RigidBody& A, const RigidBody& B, Manifold& m) {
    Vec4 uA[3], uB[3];
    AxesOf(A.Orientation, uA);
    AxesOf(B.Orientation, uB);
    float ha[4], hb[4];
    A.Shape.Extents.Store(ha);
    B.Shape.Extents.Store(hb);

    const Vec4 t = B.Position - A.Position;

    // ac = |B's axes expressed in A's frame|; the epsilon guards the
    // parallel-edge degeneracy (cross-product axes vanish, projections stay
    // conservative instead of producing false separations).
    float ac[3][3], tA[3], tB[3];
    for (int i = 0; i < 3; ++i) {
        tA[i] = Dot3(t, uA[i]);
        tB[i] = Dot3(t, uB[i]);
        for (int j = 0; j < 3; ++j)
            ac[i][j] = std::fabs(Dot3(uA[i], uB[j])) + 1e-5f;
    }

    // --- 6 face axes. Any positive separation -> disjoint. ---
    float sepA = -FLT_MAX, sepB = -FLT_MAX;
    int   axisA = -1, axisB = -1;
    for (int i = 0; i < 3; ++i) {
        const float sep = std::fabs(tA[i])
                        - (ha[i] + hb[0] * ac[i][0] + hb[1] * ac[i][1] + hb[2] * ac[i][2]);
        if (sep > 0.0f) return false;
        if (sep > sepA) { sepA = sep; axisA = i; }
    }
    for (int j = 0; j < 3; ++j) {
        const float sep = std::fabs(tB[j])
                        - (ha[0] * ac[0][j] + ha[1] * ac[1][j] + ha[2] * ac[2][j] + hb[j]);
        if (sep > 0.0f) return false;
        if (sep > sepB) { sepB = sep; axisB = j; }
    }
    // Prefer A's face unless B's is clearly better (relative + absolute
    // tolerance, a la Box2D): perfectly aligned stacked boxes tie exactly, and
    // without the bias the winner flips with float noise — changing every
    // FeatureId and silently discarding the warm-started impulses each flip.
    const bool  bestFaceIsA  = !(sepB > sepA * 0.98f + 0.0005f);
    const float bestFaceSep  = bestFaceIsA ? sepA : sepB;
    const int   bestFaceAxis = bestFaceIsA ? axisA : axisB;

    // --- 9 edge-cross axes. ---
    float bestEdgeSep = -FLT_MAX;
    int   bestEdgeI = -1, bestEdgeJ = -1;
    Vec4  bestEdgeAxis;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec4 axis = Cross(uA[i], uB[j]);
            const float lenSq = LengthSq3(axis);
            if (lenSq < 1e-6f) continue;                    // parallel edges: axis degenerate
            Vec4 L = axis / std::sqrt(lenSq);

            const float tl = Dot3(t, L);
            const float ra = ha[0] * std::fabs(Dot3(L, uA[0]))
                           + ha[1] * std::fabs(Dot3(L, uA[1]))
                           + ha[2] * std::fabs(Dot3(L, uA[2]));
            const float rb = hb[0] * std::fabs(Dot3(L, uB[0]))
                           + hb[1] * std::fabs(Dot3(L, uB[1]))
                           + hb[2] * std::fabs(Dot3(L, uB[2]));
            const float sep = std::fabs(tl) - (ra + rb);
            if (sep > 0.0f) return false;
            if (sep > bestEdgeSep) {
                bestEdgeSep  = sep;
                bestEdgeI    = i;
                bestEdgeJ    = j;
                bestEdgeAxis = tl < 0.0f ? -L : L;          // orient A -> B
            }
        }
    }

    // ODE-style face bias: a face manifold (up to 4 points) is far more stable
    // than a single edge contact, so the edge axis must beat the best face by
    // a clear margin before we take it.
    const bool useEdge = bestEdgeI >= 0
        && bestEdgeSep > bestFaceSep + 0.001f + 0.05f * std::fabs(bestFaceSep);

    if (useEdge) {
        // Support edge on A: the edge parallel to uA[i] furthest along the
        // normal; on B the edge furthest against it.
        Vec4 pa = A.Position, pb = B.Position;
        for (int k = 0; k < 3; ++k) {
            if (k != bestEdgeI) pa += uA[k] * (Dot3(bestEdgeAxis, uA[k]) > 0.0f ? ha[k] : -ha[k]);
            if (k != bestEdgeJ) pb += uB[k] * (Dot3(bestEdgeAxis, uB[k]) > 0.0f ? -hb[k] : hb[k]);
        }
        // Closest points of the two (infinite) edge lines.
        Vec4 da = uA[bestEdgeI], db = uB[bestEdgeJ];
        Vec4 d  = pb - pa;
        const float dDotA = Dot3(d, da), dDotB = Dot3(d, db), ab = Dot3(da, db);
        const float denom = 1.0f - ab * ab;
        const float s = denom > 1e-6f ? (dDotA - ab * dDotB) / denom : 0.0f;
        const float tt = s * ab - dDotB;
        Vec4 cA = pa + da * s;
        Vec4 cB = pb + db * tt;

        m.Normal = bestEdgeAxis;
        ContactPoint& cp = m.Points[0];
        cp = ContactPoint{};
        cp.Position    = (cA + cB) * 0.5f;
        cp.Penetration = -bestEdgeSep;
        cp.FeatureId   = 0xE000u | uint32_t(bestEdgeI << 4) | uint32_t(bestEdgeJ);
        m.Count = 1;
        return true;
    }

    // --- Face case: clip the incident face against the reference face. ---
    const bool  refIsA = bestFaceIsA;
    const Vec4* uR     = refIsA ? uA : uB;
    const Vec4* uI     = refIsA ? uB : uA;
    const float* hr    = refIsA ? ha : hb;
    const float* hi    = refIsA ? hb : ha;
    const Vec4  posR   = refIsA ? A.Position : B.Position;
    const Vec4  posI   = refIsA ? B.Position : A.Position;

    const int   ri   = bestFaceAxis;
    // Reference normal points from the reference box toward the other box.
    const float sign = (refIsA ? tA[ri] : -tB[ri]) >= 0.0f ? 1.0f : -1.0f;
    const Vec4  nRef = uR[ri] * sign;

    // Incident face: the face of the other box most anti-parallel to nRef.
    int   ii = 0;
    float bestDot = -FLT_MAX;
    for (int k = 0; k < 3; ++k) {
        const float dk = std::fabs(Dot3(nRef, uI[k]));
        if (dk > bestDot) { bestDot = dk; ii = k; }
    }
    const float signI = Dot3(nRef, uI[ii]) > 0.0f ? -1.0f : 1.0f;

    const int k1 = (ii + 1) % 3, k2 = (ii + 2) % 3;
    Vec4 faceCenter = posI + uI[ii] * (signI * hi[ii]);
    ClipVert poly[8], buf[8];
    poly[0] = { faceCenter - uI[k1] * hi[k1] - uI[k2] * hi[k2], 0 };
    poly[1] = { faceCenter + uI[k1] * hi[k1] - uI[k2] * hi[k2], 1 };
    poly[2] = { faceCenter + uI[k1] * hi[k1] + uI[k2] * hi[k2], 2 };
    poly[3] = { faceCenter - uI[k1] * hi[k1] + uI[k2] * hi[k2], 3 };
    int count = 4;

    // Clip against the reference face's 4 side planes.
    const int r1 = (ri + 1) % 3, r2 = (ri + 2) % 3;
    const int sideAxes[2] = { r1, r2 };
    uint32_t planeIdx = 0;
    for (int sideIdx = 0; sideIdx < 2; ++sideIdx) {
        const int rk = sideAxes[sideIdx];
        for (float ps = -1.0f; ps <= 1.0f; ps += 2.0f) {
            Vec4 ns = uR[rk] * ps;
            const float offset = Dot3(posR, ns) + hr[rk];
            ClipVert* src = (planeIdx % 2 == 0) ? poly : buf;
            ClipVert* dst = (planeIdx % 2 == 0) ? buf : poly;
            count = ClipAgainstPlane(src, count, ns, offset, planeIdx, dst);
            ++planeIdx;
            if (count == 0) return false;
        }
    }
    ClipVert* clipped = (planeIdx % 2 == 0) ? poly : buf;

    // Keep the points at (or within kPersistTol above) the reference face,
    // deepest 4. The SAT already proved real overlap on every axis, so the
    // near-miss points here are the persistence margin, not false positives.
    const float faceOffset = Dot3(posR, nRef) + hr[ri];
    Hit hits[8];
    int hitCount = 0;
    for (int i = 0; i < count; ++i) {
        const float sep = Dot3(clipped[i].p, nRef) - faceOffset;
        if (sep <= kPersistTol)
            hits[hitCount++] = { clipped[i].p, -sep, clipped[i].id };
    }
    if (hitCount == 0) return false;
    SortHits(hits, hitCount);

    m.Normal = refIsA ? nRef : -nRef;                       // manifold normal is A -> B
    m.Count  = std::min(hitCount, 4);
    const uint32_t faceTag = (refIsA ? 0u : 0x8000u)
                           | uint32_t((ri * 2 + (sign > 0.0f ? 1 : 0)) << 8);
    for (uint32_t i = 0; i < m.Count; ++i) {
        ContactPoint& cp = m.Points[i];
        Vec4 p = hits[i].p;
        cp = ContactPoint{};
        cp.Position    = p;
        cp.Penetration = hits[i].pen;
        cp.FeatureId   = faceTag | (hits[i].id & 0xFFu);
    }
    return true;
}

// =============================== dispatch ==================================

bool Collide(const RigidBody& a, const RigidBody& b, Manifold& m) {
    switch (a.Shape.Type) {
    case ShapeType::Sphere:
        switch (b.Shape.Type) {
        case ShapeType::Sphere:  return SphereSphere(a, b, m);
        case ShapeType::Capsule: return SphereCapsule(a, b, m);
        case ShapeType::Box:     return SphereBox(a, b, m);
        case ShapeType::Plane:   return SpherePlane(a, b, m);
        }
        break;
    case ShapeType::Capsule:
        switch (b.Shape.Type) {
        case ShapeType::Capsule: return CapsuleCapsule(a, b, m);
        case ShapeType::Box:     return CapsuleBox(a, b, m);
        case ShapeType::Plane:   return CapsulePlane(a, b, m);
        default:                 break;                     // non-canonical order
        }
        break;
    case ShapeType::Box:
        switch (b.Shape.Type) {
        case ShapeType::Box:    return BoxBox(a, b, m);
        case ShapeType::Plane:  return BoxPlane(a, b, m);
        default:                break;                      // non-canonical order
        }
        break;
    case ShapeType::Plane:
        break;                                              // plane-plane: never collides
    }
    return false;
}

} // namespace SGE::Physics::Narrowphase
