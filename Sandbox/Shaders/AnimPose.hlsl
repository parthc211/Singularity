// GPU pose evaluation: one thread per crowd instance runs the ENTIRE pose
// pipeline — sample the baked clip at this instance's phase (lerp T/S, nlerp
// R between fixed-rate frames), propagate the joint hierarchy, and write the
// skinning palette. Matrices are handled as explicit row quadruples in the
// engine's row-vector convention (out.r[i] = a.r[i].x*b.r0 + ... — the same
// Mul the SIMD math library implements), which sidesteps HLSL matrix-majorness
// entirely.
//
// The palette buffer doubles as the globals scratch: pass 1 writes each
// joint's GLOBAL into its slot (children read their parent's slot — same
// thread, so no synchronization needed), pass 2 replaces each slot with
// inverseBind * global.

cbuffer Params : register(b0)
{
    uint  gJointCount;
    uint  gFrameCount;
    uint  gInstanceCount;
    float gDuration;
    float gFrameRate;
    float gTime;         // global clock; per-instance phase = gTime + i*offset
    float gTimeOffset;
    float _pad;
};

// Baked clip: per (frame, joint) three float4s: T.xyz, R.xyzw, S.xyz.
StructuredBuffer<float4> gClip     : register(t0);
StructuredBuffer<uint>   gParents  : register(t1);  // 0xFFFFFFFF = root
StructuredBuffer<float4> gIbm      : register(t2);  // 4 rows per joint
RWStructuredBuffer<float4> gPalette : register(u0); // 4 rows per (instance, joint)

struct Rows { float4 r0, r1, r2, r3; };

// Engine row-vector matrix product (see SGE::Math::Mul).
Rows MulRows(Rows a, Rows b)
{
    Rows o;
    o.r0 = a.r0.x * b.r0 + a.r0.y * b.r1 + a.r0.z * b.r2 + a.r0.w * b.r3;
    o.r1 = a.r1.x * b.r0 + a.r1.y * b.r1 + a.r1.z * b.r2 + a.r1.w * b.r3;
    o.r2 = a.r2.x * b.r0 + a.r2.y * b.r1 + a.r2.z * b.r2 + a.r2.w * b.r3;
    o.r3 = a.r3.x * b.r0 + a.r3.y * b.r1 + a.r3.z * b.r2 + a.r3.w * b.r3;
    return o;
}

// TRS -> rows, matching SGE::Math::ToMatrix + the scale/translation layout
// the CPU LocalMatrix uses.
Rows TrsToRows(float3 T, float4 q, float3 S)
{
    const float x = q.x, y = q.y, z = q.z, w = q.w;
    Rows o;
    o.r0 = float4(1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y), 0) * S.x;
    o.r1 = float4(2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x), 0) * S.y;
    o.r2 = float4(2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y), 0) * S.z;
    o.r3 = float4(T, 1);
    return o;
}

[numthreads(64, 1, 1)]
void CSMain(uint3 id : SV_DispatchThreadID)
{
    const uint inst = id.x;
    if (inst >= gInstanceCount) return;

    // This instance's phase inside the (looping) clip, as fixed-rate frames.
    const float t  = fmod(gTime + float(inst) * gTimeOffset, gDuration);
    const float f  = t * gFrameRate;
    const uint  f0 = min(uint(f), gFrameCount - 1);
    const uint  f1 = min(f0 + 1, gFrameCount - 1);
    const float s  = saturate(f - float(f0));

    const uint outBase = inst * gJointCount * 4;

    // Pass 1: globals into the palette slots (parent always precedes child).
    for (uint j = 0; j < gJointCount; ++j) {
        const uint a = (f0 * gJointCount + j) * 3;
        const uint b = (f1 * gJointCount + j) * 3;

        const float3 T = lerp(gClip[a + 0].xyz, gClip[b + 0].xyz, s);
        float4 qa = gClip[a + 1];
        float4 qb = gClip[b + 1];
        if (dot(qa, qb) < 0) qb = -qb;                 // shortest path
        const float4 q = normalize(lerp(qa, qb, s));   // nlerp
        const float3 S = lerp(gClip[a + 2].xyz, gClip[b + 2].xyz, s);

        Rows g = TrsToRows(T, q, S);
        const uint parent = gParents[j];
        if (parent != 0xFFFFFFFFu) {
            Rows p;
            const uint pb = outBase + parent * 4;
            p.r0 = gPalette[pb + 0]; p.r1 = gPalette[pb + 1];
            p.r2 = gPalette[pb + 2]; p.r3 = gPalette[pb + 3];
            g = MulRows(g, p);                          // global = local * parentGlobal
        }
        const uint ob = outBase + j * 4;
        gPalette[ob + 0] = g.r0; gPalette[ob + 1] = g.r1;
        gPalette[ob + 2] = g.r2; gPalette[ob + 3] = g.r3;
    }

    // Pass 2: palette = inverseBind * global, in place.
    for (uint j2 = 0; j2 < gJointCount; ++j2) {
        const uint ob = outBase + j2 * 4;
        Rows g;
        g.r0 = gPalette[ob + 0]; g.r1 = gPalette[ob + 1];
        g.r2 = gPalette[ob + 2]; g.r3 = gPalette[ob + 3];
        Rows ibm;
        const uint ib = j2 * 4;
        ibm.r0 = gIbm[ib + 0]; ibm.r1 = gIbm[ib + 1];
        ibm.r2 = gIbm[ib + 2]; ibm.r3 = gIbm[ib + 3];
        const Rows pal = MulRows(ibm, g);
        gPalette[ob + 0] = pal.r0; gPalette[ob + 1] = pal.r1;
        gPalette[ob + 2] = pal.r2; gPalette[ob + 3] = pal.r3;
    }
}
