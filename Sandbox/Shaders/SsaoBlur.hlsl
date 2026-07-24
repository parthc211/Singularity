// SSAO is noisy (per-pixel random kernel rotation), so blur it. A small box blur
// over the raw AO is enough to dissolve the noise back into smooth occlusion.
cbuffer BlurCB : register(b0)
{
    float2 gTexelSize;
    float2 _pad;
};

Texture2D    gAO      : register(t0);
SamplerState gSampler : register(s0);

struct VSOutput { float4 Pos : SV_POSITION; float2 UV : TEXCOORD0; };

VSOutput VSMain(uint id : SV_VertexID)
{
    VSOutput o;
    o.UV  = float2((id << 1) & 2, id & 2);
    o.Pos = float4(o.UV * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    return o;
}

float PSMain(VSOutput i) : SV_TARGET
{
    float sum = 0.0;
    [unroll] for (int y = -2; y <= 1; ++y)
        for (int x = -2; x <= 1; ++x)
            sum += gAO.Sample(gSampler, i.UV + float2(x, y) * gTexelSize).r;
    return sum / 16.0; // 4x4 box
}
