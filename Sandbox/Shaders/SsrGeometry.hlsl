// SsrGeometry.hlsl — the G-buffer for screen-space reflections. Everything is
// stored in VIEW space (eye at origin, +z forward), which is the natural space
// to ray-march a reflection and project it back to the screen. Three targets:
//   RT0 = lit colour (rgb) + reflectivity (a)  — what reflections sample, and how
//                                                 mirror-like this surface is
//   RT1 = view-space normal
//   RT2 = view-space position
// Lighting uses a view-space light direction (the camera's view rotation applied
// on the CPU), so it stays stable as the camera moves.

cbuffer Object : register(b0)
{
    float4x4 gMVP;
    float4x4 gModelView;
    float4   gAlbedoReflect;   // rgb albedo, a reflectivity
}
cbuffer Frame : register(b1)
{
    float4 gViewLightDir;      // light direction in VIEW space
}

struct VSOut {
    float4 Pos       : SV_POSITION;
    float3 NormalV   : NORMAL;
    float3 PosV      : TEXCOORD0;
};
struct PSOut {
    float4 Color  : SV_TARGET0;   // rgb lit, a reflectivity
    float4 Normal : SV_TARGET1;   // view-space normal
    float4 PosV   : SV_TARGET2;   // view-space position
};

VSOut VSMain(float3 pos : POSITION, float3 nrm : NORMAL)
{
    VSOut o;
    o.Pos     = mul(gMVP, float4(pos, 1.0));
    o.PosV    = mul(gModelView, float4(pos, 1.0)).xyz;
    o.NormalV = mul((float3x3)gModelView, nrm);
    return o;
}

PSOut PSMain(VSOut i)
{
    const float3 N = normalize(i.NormalV);
    const float  ndl = saturate(dot(N, -gViewLightDir.xyz));
    const float3 lit = gAlbedoReflect.rgb * (0.1 + 0.9 * ndl);

    PSOut o;
    o.Color  = float4(lit, gAlbedoReflect.a);
    o.Normal = float4(N, 0.0);
    o.PosV   = float4(i.PosV, 1.0);
    return o;
}
