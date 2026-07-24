// Separable Gaussian blur. The same shader runs horizontally then vertically
// (direction picked by the cbuffer) — a 2D blur done as two cheap 1D passes.
cbuffer Post : register(b0)
{
    float2 gTexelSize;
    float  gDirection; // 0 = horizontal, 1 = vertical
    float  gThreshold; // unused
    float  gIntensity; // unused
    float  gExposure;  // unused
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
    const float w[5] = { 0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216 }; // 9-tap Gaussian
    float2 step = (gDirection < 0.5) ? float2(gTexelSize.x, 0.0) : float2(0.0, gTexelSize.y);

    float3 sum = gInput.Sample(gSampler, i.UV).rgb * w[0];
    [unroll] for (int k = 1; k < 5; ++k) {
        sum += gInput.Sample(gSampler, i.UV + step * k).rgb * w[k];
        sum += gInput.Sample(gSampler, i.UV - step * k).rgb * w[k];
    }
    return float4(sum, 1.0);
}
