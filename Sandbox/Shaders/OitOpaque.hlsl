// OitOpaque.hlsl — plain lit opaque geometry, drawn first (writes colour + depth).
// The transparent passes test against this depth so transparency behind opaque
// geometry is correctly occluded.

cbuffer Object : register(b0)
{
    float4x4 gMVP;
    float4x4 gModel;
    float4x4 gModelView;
    float4   gColor;
}

struct VSOut { float4 Pos : SV_POSITION; float3 N : NORMAL; };

VSOut VSMain(float3 pos : POSITION, float3 nrm : NORMAL)
{
    VSOut o;
    o.Pos = mul(gMVP, float4(pos, 1.0));
    o.N   = mul((float3x3)gModel, nrm);
    return o;
}

float4 PSMain(VSOut i) : SV_TARGET
{
    const float3 L = normalize(float3(0.4, 0.8, -0.4));
    const float  ndl = saturate(dot(normalize(i.N), L));
    return float4(gColor.rgb * (0.15 + 0.85 * ndl), 1.0);
}
