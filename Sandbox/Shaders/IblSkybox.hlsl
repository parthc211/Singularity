// IblSkybox.hlsl — draw the environment cubemap as the background. A fullscreen
// triangle whose pixels reconstruct a world-space view ray (via the inverse
// view-projection) and sample the env cube along it. Drawn first with no depth,
// so scene geometry overwrites it.

cbuffer Sky : register(b0)
{
    float4x4 gInvViewProj;
    float4   gCamPos;
    float    gExposure;   // shared tonemap exposure with the objects
    float3   _pad;
}

TextureCube<float4> gEnv  : register(t0);
SamplerState        gSamp : register(s0);

struct VSOut {
    float4 Pos : SV_POSITION;
    float2 Ndc : TEXCOORD0;
};

VSOut VSMain(uint vid : SV_VertexID)
{
    // Oversized triangle covering the screen; z at the far plane.
    float2 uv = float2((vid << 1) & 2, vid & 2);
    VSOut o;
    o.Ndc = uv * 2.0 - 1.0;
    o.Pos = float4(o.Ndc, 1.0, 1.0);
    return o;
}

float4 PSMain(VSOut i) : SV_TARGET
{
    float4 world = mul(gInvViewProj, float4(i.Ndc, 1.0, 1.0));
    const float3 dir = normalize(world.xyz / world.w - gCamPos.xyz);
    float3 color = gEnv.SampleLevel(gSamp, dir, 0).rgb;
    // ACES-ish tonemap + gamma so the HDR sky fits the UNORM back buffer.
    color *= gExposure;
    color = (color * (2.51 * color + 0.03)) / (color * (2.43 * color + 0.59) + 0.14);
    color = pow(saturate(color), 1.0 / 2.2);
    return float4(color, 1.0);
}
