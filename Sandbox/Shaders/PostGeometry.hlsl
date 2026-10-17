// PostGeometry.hlsl — the scene the post stack operates on. Lambert-lit spheres
// with per-object emissive, rendered to an HDR (RGBA16F) target in LINEAR light
// (no tonemap, no gamma — that is the composite pass's job). Bright emissive
// spheres give bloom and auto-exposure something to react to.

cbuffer Object : register(b0)
{
    float4x4 gMVP;
    float4x4 gModel;
    float4   gAlbedo;
    float4   gEmissive;   // rgb emissive radiance (can exceed 1.0)
}

struct VSOut {
    float4 Pos    : SV_POSITION;
    float3 Normal : NORMAL;
};

VSOut VSMain(float3 pos : POSITION, float3 nrm : NORMAL)
{
    VSOut o;
    o.Pos    = mul(gMVP, float4(pos, 1.0));
    o.Normal = mul((float3x3)gModel, nrm);
    return o;
}

float4 PSMain(VSOut i) : SV_TARGET
{
    const float3 N = normalize(i.Normal);
    const float3 L = normalize(float3(0.3, 0.7, -0.5));
    const float  ndl = saturate(dot(N, L));
    float3 color = gAlbedo.rgb * (0.08 + 0.92 * ndl) + gEmissive.rgb;
    return float4(color, 1.0);
}
