// Crowd rendering: ONE instanced draw for the whole grid. The vertex shader
// pulls this instance's skinning palette (computed by AnimPose.hlsl) from a
// structured buffer via SV_InstanceID and derives the grid placement from the
// instance index — no per-instance CPU work, no per-instance constants.
//
// Palette matrices are row quadruples in the engine's row-vector convention:
// v' = v.x*r0 + v.y*r1 + v.z*r2 + r3.

cbuffer Frame : register(b0)
{
    float4x4 gViewProj;    // row-major upload; mul(gViewProj, v) = v * VP
    float4   gLightDir;
    float4   gCameraPos;
    float4   gTint;
    uint     gJointCount;
    uint     gGridN;
    float    gSpacing;
    float    gScale;
    float    gYaw;
    float3   _pad;
};

StructuredBuffer<float4> gPalette : register(t0);   // 4 rows per (instance, joint)

struct VSInput {
    float3 Position : POSITION;
    float3 Normal   : NORMAL;
    float2 TexCoord : TEXCOORD;
    float4 Tangent  : TANGENT;
    uint4  Joints   : BLENDINDICES;
    float4 Weights  : BLENDWEIGHT;
    uint   Instance : SV_InstanceID;
};
struct VSOutput {
    float4 Pos      : SV_POSITION;
    float3 WorldPos : TEXCOORD1;
    float3 Normal   : NORMAL;
};

VSOutput VSMain(VSInput input)
{
    // Weighted palette rows: blending matrices first is identical to blending
    // transformed positions (linearity), and cheaper.
    const uint base = input.Instance * gJointCount * 4;
    float4 r0 = 0, r1 = 0, r2 = 0, r3 = 0;
    [unroll]
    for (int k = 0; k < 4; ++k) {
        const uint  o = base + input.Joints[k] * 4;
        const float w = input.Weights[k];
        r0 += gPalette[o + 0] * w;
        r1 += gPalette[o + 1] * w;
        r2 += gPalette[o + 2] * w;
        r3 += gPalette[o + 3] * w;
    }
    const float3 skinned = input.Position.x * r0.xyz + input.Position.y * r1.xyz
                         + input.Position.z * r2.xyz + r3.xyz;
    const float3 skinnedN = input.Normal.x * r0.xyz + input.Normal.y * r1.xyz
                          + input.Normal.z * r2.xyz;

    // Grid placement from the instance index (matches the CPU layout).
    const float origin = -gSpacing * float(gGridN - 1) * 0.5;
    const float3 slot  = float3(origin + gSpacing * float(input.Instance % gGridN), 0,
                                origin + gSpacing * float(input.Instance / gGridN));

    // scale, yaw, translate — same order (and rotation sense: row-vector
    // XMMatrixRotationY) as the CPU instance matrix.
    const float cs = cos(gYaw), sn = sin(gYaw);
    float3 p = skinned * gScale;
    p = float3(p.x * cs + p.z * sn, p.y, -p.x * sn + p.z * cs) + slot;
    float3 n = float3(skinnedN.x * cs + skinnedN.z * sn, skinnedN.y,
                      -skinnedN.x * sn + skinnedN.z * cs);

    VSOutput o;
    o.Pos      = mul(gViewProj, float4(p, 1.0));
    o.WorldPos = p;
    o.Normal   = n;
    return o;
}

float4 PSMain(VSOutput i) : SV_TARGET
{
    const float3 N = normalize(i.Normal);
    const float3 L = normalize(-gLightDir.xyz);
    const float3 V = normalize(gCameraPos.xyz - i.WorldPos);
    const float3 H = normalize(L + V);

    const float ndl  = saturate(dot(N, L));
    const float back = saturate(dot(N, -L)) * 0.15;
    const float spec = pow(saturate(dot(N, H)), 32.0) * 0.25;

    float3 color = gTint.rgb * (0.18 + ndl * 0.9 + back) + spec * ndl;
    return float4(pow(color, 1.0 / 2.2), 1.0);
}
