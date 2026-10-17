// PostBlur.hlsl — one axis of a separable Gaussian blur. Run horizontally then
// vertically (ping-ponging two targets) to approximate a 2D blur cheaply. The
// blur direction + texel size arrives as gTexelDir.

cbuffer Params : register(b0)
{
    float2 gTexelDir;   // (1/width, 0) horizontal, or (0, 1/height) vertical
    float2 _pad;
}

Texture2D    gInput : register(t0);
SamplerState gSamp  : register(s0);

struct VSOut { float4 Pos : SV_POSITION; float2 Uv : TEXCOORD0; };

VSOut VSMain(uint vid : SV_VertexID)
{
    float2 uv = float2((vid << 1) & 2, vid & 2);
    VSOut o; o.Uv = uv; o.Pos = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    o.Pos.y = -o.Pos.y;
    return o;
}

float4 PSMain(VSOut i) : SV_TARGET
{
    // 9-tap Gaussian (normalized weights).
    const float w[5] = { 0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216 };
    float3 sum = gInput.SampleLevel(gSamp, i.Uv, 0).rgb * w[0];
    [unroll] for (int k = 1; k < 5; ++k) {
        const float2 off = gTexelDir * float(k);
        sum += gInput.SampleLevel(gSamp, i.Uv + off, 0).rgb * w[k];
        sum += gInput.SampleLevel(gSamp, i.Uv - off, 0).rgb * w[k];
    }
    return float4(sum, 1.0);
}
