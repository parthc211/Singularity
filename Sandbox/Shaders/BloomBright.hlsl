// Bright-pass: keep only the part of the HDR scene above a luminance threshold —
// that's what will be blurred into a glow.
cbuffer Post : register(b0)
{
    float2 gTexelSize;
    float  gDirection; // unused here
    float  gThreshold;
    float  gIntensity; // unused here
    float  gExposure;  // unused here
    float2 _pad;
};

Texture2D    gInput  : register(t0);
SamplerState gSampler : register(s0);

struct VSOutput { float4 Pos : SV_POSITION; float2 UV : TEXCOORD0; };

VSOutput VSMain(uint id : SV_VertexID)
{
    VSOutput o;
    o.UV  = float2((id << 1) & 2, id & 2);
    o.Pos = float4(o.UV * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    return o;
}

float4 PSMain(VSOutput i) : SV_TARGET
{
    float3 c   = gInput.Sample(gSampler, i.UV).rgb;
    float  lum = dot(c, float3(0.2126, 0.7152, 0.0722));
    // Soft knee: scale toward zero as luminance approaches the threshold.
    float  k   = saturate((lum - gThreshold) / max(lum, 1e-4));
    return float4(c * k, 1.0);
}
