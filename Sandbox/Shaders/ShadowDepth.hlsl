// Shadow pass: render scene depth from the LIGHT's point of view into the shadow
// map. Depth-only — there is no pixel shader; the rasterizer just writes Z.
cbuffer ShadowObject : register(b0)
{
    float4x4 gLightMVP; // model * lightView * lightProj
};

struct VSInput { float3 Position : POSITION; };

float4 VSMain(VSInput input) : SV_POSITION
{
    return mul(gLightMVP, float4(input.Position, 1.0)); // mul(M,v) per engine convention
}
