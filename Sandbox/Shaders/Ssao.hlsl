// Screen-space ambient occlusion. For each pixel, sample a hemisphere of points
// around its world position (oriented to the surface normal); for each, project
// to screen and read the G-buffer position there. If that stored geometry is
// closer to the camera than the sample point, the sample is occluded. The
// fraction occluded = how much ambient light is blocked in nearby creases.

cbuffer SsaoCB : register(b0)
{
    float4x4 gViewProj;
    float4   gCameraPos;  // .xyz
    float    gRadius;
    float    gBias;
    float    gPower;
    float    gStrength;
    float2   gScreen;     // pixels
    int      gSampleCount;
    float    _pad;
    float4   gKernel[32]; // tangent-space hemisphere (z+ = normal)
};

Texture2D    gNormal   : register(t0); // world-space normal (G-buffer)
Texture2D    gPosition : register(t1); // world-space position (G-buffer)
SamplerState gSampler  : register(s0); // point clamp

struct VSOutput { float4 Pos : SV_POSITION; float2 UV : TEXCOORD0; };

VSOutput VSMain(uint id : SV_VertexID)
{
    VSOutput o;
    o.UV  = float2((id << 1) & 2, id & 2);
    o.Pos = float4(o.UV * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    return o;
}

float Hash21(float2 p) {
    p = frac(p * float2(123.34, 456.21));
    p += dot(p, p + 45.32);
    return frac(p.x * p.y);
}

float PSMain(VSOutput i) : SV_TARGET
{
    float3 N = gNormal.Sample(gSampler, i.UV).xyz;
    if (dot(N, N) < 0.01) return 1.0; // background: fully lit (no occlusion)
    N = normalize(N);
    float3 P = gPosition.Sample(gSampler, i.UV).xyz;

    // Per-pixel random rotation of the kernel (broken up later by the blur).
    float  ang = Hash21(i.UV * gScreen) * 6.28318;
    float3 rnd = float3(cos(ang), sin(ang), 0.0);
    float3 T   = normalize(rnd - N * dot(rnd, N)); // Gram-Schmidt onto the tangent plane
    float3 B   = cross(N, T);
    float3x3 TBN = float3x3(T, B, N);              // rows: tangent, bitangent, normal

    float occlusion = 0.0;
    [loop] for (int s = 0; s < gSampleCount; ++s) {
        float3 samplePos = P + mul(gKernel[s].xyz, TBN) * gRadius; // tangent -> world

        float4 clip = mul(gViewProj, float4(samplePos, 1.0));
        if (clip.w <= 0.0) continue;
        float2 suv = (clip.xy / clip.w) * float2(0.5, -0.5) + 0.5;
        if (any(suv < 0.0) || any(suv > 1.0)) continue;

        float3 occluder    = gPosition.Sample(gSampler, suv).xyz;
        float  occluderD   = length(occluder  - gCameraPos.xyz);
        float  sampleD     = length(samplePos - gCameraPos.xyz);
        // Only count occluders within radius of the pixel (avoids dark halos).
        float  rangeCheck  = smoothstep(0.0, 1.0, gRadius / max(distance(P, occluder), 1e-4));
        if (occluderD < sampleD - gBias)
            occlusion += rangeCheck;
    }

    float ao = 1.0 - (occlusion / gSampleCount) * gStrength;
    return pow(saturate(ao), gPower);
}
