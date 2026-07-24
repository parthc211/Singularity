// Composite: add the blurred bloom back onto the HDR scene, apply exposure, then
// tonemap HDR -> [0,1] for the 8-bit back buffer.
cbuffer Post : register(b0)
{
    float2 gTexelSize; // unused
    float  gDirection; // unused
    float  gThreshold; // unused
    float  gIntensity; // bloom strength
    float  gExposure;
    float2 _pad;
};

Texture2D    gScene   : register(t0);
Texture2D    gBloom   : register(t1);
SamplerState gSampler : register(s0);

struct VSOutput { float4 Pos : SV_POSITION; float2 UV : TEXCOORD0; };

VSOutput VSMain(uint id : SV_VertexID)
{
    VSOutput o;
    o.UV  = float2((id << 1) & 2, id & 2);
    o.Pos = float4(o.UV * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    return o;
}

// ACES filmic tonemap approximation (Narkowicz).
float3 ACESFilm(float3 x)
{
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return saturate((x * (a * x + b)) / (x * (c * x + d) + e));
}

float4 PSMain(VSOutput i) : SV_TARGET
{
    float3 scene = gScene.Sample(gSampler, i.UV).rgb;
    float3 bloom = gBloom.Sample(gSampler, i.UV).rgb;
    float3 hdr   = (scene + bloom * gIntensity) * gExposure;
    return float4(ACESFilm(hdr), 1.0);
}
