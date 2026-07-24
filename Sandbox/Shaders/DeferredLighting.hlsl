// Deferred lighting pass: one fullscreen triangle that reads the G-buffer and
// accumulates every light per pixel. Cost is (screen pixels x lights), paid once
// regardless of scene complexity or overdraw — the whole point of deferred.

#define MAX_LIGHTS 64

// Must match SGE::GpuLight (DeferredScene.cpp). 32 bytes, 16-byte aligned.
struct GpuLight
{
    float3 Position; float Radius;
    float3 Color;    float Intensity;
};

// Must match SGE::LightData (DeferredScene.cpp).
cbuffer LightData : register(b0)
{
    float4   CameraPos;  // .xyz world-space eye (for specular)
    float4   Ambient;    // .rgb ambient term
    uint     LightCount;
    uint     DebugMode;  // 0 composite, 1 albedo, 2 normal, 3 position
    uint2    _pad;
    GpuLight Lights[MAX_LIGHTS];
};

Texture2D    gAlbedo   : register(t0);
Texture2D    gNormal   : register(t1);
Texture2D    gPosition : register(t2);
SamplerState gSampler  : register(s0);

struct VSOutput
{
    float4 Pos : SV_POSITION;
    float2 UV  : TEXCOORD0;
};

// Fullscreen triangle from the vertex id alone — no vertex buffer. The 3 verts
// produce UVs (0,0),(2,0),(0,2); the oversized triangle covers the screen and
// the rasterizer clips it, giving correct [0,1] UVs across the viewport.
VSOutput VSMain(uint id : SV_VertexID)
{
    VSOutput o;
    o.UV  = float2((id << 1) & 2, id & 2);
    o.Pos = float4(o.UV * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    return o;
}

float4 PSMain(VSOutput input) : SV_TARGET
{
    float3 albedo   = gAlbedo.Sample(gSampler, input.UV).rgb;
    float3 N        = gNormal.Sample(gSampler, input.UV).xyz;
    float3 worldPos = gPosition.Sample(gSampler, input.UV).xyz;

    // Pixels with no geometry have a zero normal (cleared target) -> draw sky.
    float nlen = length(N);
    if (nlen < 0.1)
        return float4(0.04, 0.05, 0.08, 1.0);
    N /= nlen;

    // G-buffer debug visualizations.
    if (DebugMode == 1) return float4(albedo, 1.0);
    if (DebugMode == 2) return float4(N * 0.5 + 0.5, 1.0);
    if (DebugMode == 3) return float4(frac(worldPos), 1.0);

    float3 V   = normalize(CameraPos.xyz - worldPos);
    float3 lit = Ambient.rgb * albedo;

    [loop]
    for (uint l = 0; l < LightCount; ++l)
    {
        float3 toLight = Lights[l].Position - worldPos;
        float  dist    = length(toLight);
        float3 L       = toLight / max(dist, 1e-4);

        // Smooth radius-based attenuation (squared falloff to zero at Radius).
        float atten = saturate(1.0 - dist / max(Lights[l].Radius, 1e-4));
        atten *= atten;

        float  ndotl    = saturate(dot(N, L));
        float3 radiance = Lights[l].Color * Lights[l].Intensity * atten;

        float3 H    = normalize(L + V);
        float  spec = pow(saturate(dot(N, H)), 32.0);

        lit += radiance * (albedo * ndotl + spec * 0.3);
    }

    return float4(lit, 1.0);
}
