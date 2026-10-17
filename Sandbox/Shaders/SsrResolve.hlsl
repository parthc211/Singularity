// SsrResolve.hlsl — screen-space reflections, composited to the back buffer.
//
// For each reflective pixel it reflects the view ray about the surface normal,
// then marches that reflection in SCREEN space from the pixel toward the ray's
// far end. Marching in screen space (rather than view space) keeps the sample
// spacing even across the image; view-space depth is reconstructed per step with
// perspective-correct interpolation (1/z is linear in screen space). A depth
// BIAS rejects the grazing self-intersections that plague flat floors, and the
// hit "thickness" grows with the per-step depth gap so fast-receding rays don't
// skip a surface. A binary refinement then pins the exact crossing.
//
// This is the cheap screen-space approximation: it can only reflect what is on
// screen, so reflections fade out where the ray would leave the frame.

cbuffer Params : register(b0)
{
    float4x4 gProj;
    float2   gScreen;
    int      gSteps;
    float    gReach;      // reflection ray length scale
    float    gThickness;  // minimum "behind a surface" band for a hit
    float    gStrength;   // overall reflection intensity
    float    gEdgeFade;
    float    _pad;
}

Texture2D    gColor  : register(t0);   // rgb lit, a reflectivity
Texture2D    gNormal : register(t1);   // view-space normal
Texture2D    gPosV   : register(t2);   // view-space position
SamplerState gSamp   : register(s0);

struct VSOut { float4 Pos : SV_POSITION; float2 Uv : TEXCOORD0; };

VSOut VSMain(uint vid : SV_VertexID)
{
    float2 uv = float2((vid << 1) & 2, vid & 2);
    VSOut o; o.Uv = uv; o.Pos = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    o.Pos.y = -o.Pos.y;
    return o;
}

// View-space point -> screen UV.
float2 ViewToUv(float3 vp)
{
    float4 clip = mul(gProj, float4(vp, 1.0));
    float2 ndc = clip.xy / max(clip.w, 1e-4);
    return float2(ndc.x * 0.5 + 0.5, -ndc.y * 0.5 + 0.5);
}

float4 PSMain(VSOut i) : SV_TARGET
{
    const float4 col   = gColor.SampleLevel(gSamp, i.Uv, 0);
    const float3 scene = col.rgb;
    const float  refl  = col.a;
    float3 outc = scene;

    const float3 P = gPosV.SampleLevel(gSamp, i.Uv, 0).xyz;   // view-space surface
    if (refl > 0.001 && P.z > 1e-3) {
        const float3 N = normalize(gNormal.SampleLevel(gSamp, i.Uv, 0).xyz);
        const float3 V = normalize(P);          // eye -> surface
        const float3 R = reflect(V, N);

        // Ray end, clipped to stay in front of the near plane.
        float maxDist = gReach;
        if (R.z < 0.0) maxDist = min(maxDist, (0.05 - P.z) / R.z);
        const float3 Q = P + R * max(maxDist, 0.0);

        const float2 uvA  = ViewToUv(P);
        const float2 uvB  = ViewToUv(Q);
        const float  invA = 1.0 / P.z, invB = 1.0 / Q.z;
        const float  bias = 0.03;                // kills grazing self-hits

        // Detect a CROSSING (ray goes from in-front-of to behind a surface)
        // rather than accepting any hit within a thickness band — the band
        // approach makes neighbouring pixels trigger at different march steps,
        // which shows as "stacked layer" rings. The binary refinement then
        // converges to the TRUE crossing regardless of which step found it, so a
        // deterministic half-step offset stays smooth without the noise a
        // per-pixel jitter would add.
        const float stepT  = 1.0 / float(gSteps);
        const float jitter = 0.5;

        bool   hit   = false;
        float2 hitUv = 0.0;
        float  tPrev = 0.0;
        float  dPrev = -bias;                    // start just in front of the surface

        [loop] for (int s = 0; s < gSteps; ++s) {
            const float  t  = stepT * (float(s) + jitter);
            if (t > 1.0) break;
            const float2 uv = lerp(uvA, uvB, t);
            if (any(uv < 0.0) || any(uv > 1.0)) break;

            const float rayZ   = 1.0 / lerp(invA, invB, t);   // perspective-correct
            const float sceneZ = gPosV.SampleLevel(gSamp, uv, 0).z;
            const float d = (sceneZ > 1e-3) ? (rayZ - sceneZ) : -1e9; // <0 in front

            if (dPrev < 0.0 && d >= 0.0) {
                // Binary-refine the exact crossing between tPrev and t.
                float lo = tPrev, hi = t;
                [unroll] for (int r = 0; r < 6; ++r) {
                    const float mt = 0.5 * (lo + hi);
                    const float mZ = 1.0 / lerp(invA, invB, mt);
                    const float sZ = gPosV.SampleLevel(gSamp, lerp(uvA, uvB, mt), 0).z;
                    if (sZ > 1e-3 && mZ - sZ > 0.0) hi = mt; else lo = mt;
                }
                const float2 huv = lerp(uvA, uvB, hi);
                const float  hz  = 1.0 / lerp(invA, invB, hi);
                const float  hs  = gPosV.SampleLevel(gSamp, huv, 0).z;
                // Reject silhouette "hits" where the real surface is far behind
                // (the ray actually passed over a thin edge, not into a surface).
                if (hz - hs < gThickness) { hitUv = huv; hit = true; }
                break;
            }
            dPrev = d; tPrev = t;
        }

        if (hit) {
            const float  fres = pow(1.0 - saturate(dot(N, -V)), 5.0);
            const float2 e = smoothstep(0.0, gEdgeFade, hitUv) *
                             (1.0 - smoothstep(1.0 - gEdgeFade, 1.0, hitUv));
            const float  fade = e.x * e.y;
            const float3 reflColor = gColor.SampleLevel(gSamp, hitUv, 0).rgb;
            const float  w = saturate(refl * (0.35 + 0.65 * fres) * fade * gStrength);
            outc = lerp(scene, reflColor, w);
        }
    }

    return float4(pow(saturate(outc), 1.0 / 2.2), 1.0);
}
