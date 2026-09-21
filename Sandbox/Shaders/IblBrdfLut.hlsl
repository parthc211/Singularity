// IblBrdfLut.hlsl — the environment BRDF lookup, the second half of the
// split-sum approximation (compute). Precomputes, per (NdotV, roughness), the
// scale (A) and bias (B) so specular IBL = prefiltered * (F0*A + B). Independent
// of the environment, so it's baked once. Output is an RG16F 2D texture; x axis
// = NdotV in [0,1], y axis = roughness in [0,1].

cbuffer Params : register(b0) { uint gSize; }

RWTexture2D<float2> gOut : register(u0);

static const float PI = 3.14159265359;

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

// Smith geometry with the IBL k = a^2 / 2 (not the direct-light k).
float GeometrySchlickGGX(float NoV, float roughness)
{
    const float k = (roughness * roughness) / 2.0;
    return NoV / (NoV * (1.0 - k) + k);
}
float GeometrySmith(float NoV, float NoL, float roughness)
{
    return GeometrySchlickGGX(NoV, roughness) * GeometrySchlickGGX(NoL, roughness);
}

[numthreads(8, 8, 1)]
void CSMain(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= gSize || id.y >= gSize) return;
    const float NoV       = max((float(id.x) + 0.5) / float(gSize), 1e-3);
    const float roughness = (float(id.y) + 0.5) / float(gSize);

    const float3 V = float3(sqrt(1.0 - NoV * NoV), 0.0, NoV); // in a frame where N = +Z
    const float3 N = float3(0.0, 0.0, 1.0);

    const uint kSamples = 1024u;
    float A = 0.0, B = 0.0;
    for (uint i = 0; i < kSamples; ++i) {
        const float2 Xi = Hammersley(i, kSamples);
        const float3 H  = ImportanceSampleGGX(Xi, N, roughness);
        const float3 L  = normalize(2.0 * dot(V, H) * H - V);

        const float NoL = saturate(L.z);
        const float NoH = saturate(H.z);
        const float VoH = saturate(dot(V, H));
        if (NoL <= 0.0) continue;

        const float G     = GeometrySmith(NoV, NoL, roughness);
        const float G_Vis = (G * VoH) / max(NoH * NoV, 1e-6);
        const float Fc    = pow(1.0 - VoH, 5.0);
        A += (1.0 - Fc) * G_Vis;
        B += Fc * G_Vis;
    }
    gOut[id.xy] = float2(A, B) / float(kSamples);
}
