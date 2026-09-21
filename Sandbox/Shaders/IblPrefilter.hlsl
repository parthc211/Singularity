// IblPrefilter.hlsl — specular IBL bake (compute), the prefiltered environment
// half of the split-sum approximation. Each mip of the output cube holds the
// environment convolved with the GGX lobe for one roughness (mip 0 = mirror,
// last mip = fully rough). At shade time: prefiltered.SampleLevel(R, roughness*maxMip).
//
// GGX importance sampling concentrates samples in the lobe; sampling the env at
// a mip chosen from the sample density kills fireflies from the bright sun.

cbuffer Params : register(b0)
{
    uint  gFaceSize;    // this mip's face size
    float gRoughness;   // roughness for this mip
    uint  gEnvSize;     // env cube base face size (for the mip-selection heuristic)
    uint  gSampleCount;
}

TextureCube<float4>      gEnv : register(t0);
RWTexture2DArray<float4> gOut : register(u0);
SamplerState             gSamp : register(s0);

static const float PI = 3.14159265359;

float3 DirFromFaceUV(uint face, float2 uv)
{
    float2 st = uv * 2.0 - 1.0;
    float3 d;
    if      (face == 0) d = float3( 1.0, -st.y, -st.x);
    else if (face == 1) d = float3(-1.0, -st.y,  st.x);
    else if (face == 2) d = float3( st.x,  1.0,  st.y);
    else if (face == 3) d = float3( st.x, -1.0, -st.y);
    else if (face == 4) d = float3( st.x, -st.y,  1.0);
    else                d = float3(-st.x, -st.y, -1.0);
    return normalize(d);
}

// Low-discrepancy sequence (Van der Corput radical inverse in base 2).
float RadicalInverseVdC(uint bits)
{
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10;
}
float2 Hammersley(uint i, uint n) { return float2(float(i) / float(n), RadicalInverseVdC(i)); }

// Sample the GGX distribution: a half-vector H in world space around N.
float3 ImportanceSampleGGX(float2 Xi, float3 N, float roughness)
{
    const float a = roughness * roughness;
    const float phi      = 2.0 * PI * Xi.x;
    const float cosTheta = sqrt((1.0 - Xi.y) / (1.0 + (a * a - 1.0) * Xi.y));
    const float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

    const float3 h = float3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
    float3 up = abs(N.z) < 0.999 ? float3(0, 0, 1) : float3(1, 0, 0);
    const float3 tx = normalize(cross(up, N));
    const float3 ty = cross(N, tx);
    return normalize(tx * h.x + ty * h.y + N * h.z);
}

[numthreads(8, 8, 1)]
void CSMain(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= gFaceSize || id.y >= gFaceSize) return;
    const float2 uv = (float2(id.xy) + 0.5) / float(gFaceSize);
    const float3 N  = DirFromFaceUV(id.z, uv);
    const float3 R  = N;
    const float3 V  = N;   // split-sum assumption: view == reflection == normal

    const float a = gRoughness * gRoughness;
    float3 color  = 0.0;
    float  weight = 0.0;
    for (uint i = 0; i < gSampleCount; ++i) {
        const float2 Xi = Hammersley(i, gSampleCount);
        const float3 H  = ImportanceSampleGGX(Xi, N, gRoughness);
        const float3 L  = normalize(2.0 * dot(V, H) * H - V);
        const float  NoL = dot(N, L);
        if (NoL <= 0.0) continue;

        // Pick an env mip from the sample's solid angle vs a texel's, so bright
        // pixels don't alias into fireflies (Chetan Jags / Epic technique).
        const float NoH = saturate(dot(N, H));
        const float VoH = saturate(dot(V, H));
        const float D   = (a * a) / max(PI * pow(NoH * NoH * (a * a - 1.0) + 1.0, 2.0), 1e-6);
        const float pdf = (D * NoH / max(4.0 * VoH, 1e-6)) + 1e-4;
        const float saTexel  = 4.0 * PI / (6.0 * gEnvSize * gEnvSize);
        const float saSample = 1.0 / (float(gSampleCount) * pdf);
        const float mip = (gRoughness == 0.0) ? 0.0 : max(0.5 * log2(saSample / saTexel), 0.0);

        color  += gEnv.SampleLevel(gSamp, L, mip).rgb * NoL;
        weight += NoL;
    }
    color /= max(weight, 1e-4);
    gOut[uint3(id.xy, id.z)] = float4(color, 1.0);
}
