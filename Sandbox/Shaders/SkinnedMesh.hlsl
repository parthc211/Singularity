// GPU skinning: each vertex blends up to 4 bone-palette matrices
// (palette[i] = inverseBind[i] * animatedGlobal[i], built on the CPU each
// frame) and the blended model-space result goes through the usual MVP.
//
// Matrices arrive row-major from the engine; HLSL reads cbuffers column-major,
// so mul(M, v) here computes v * M in engine convention — same trick as every
// other shader in the project.

#define MAX_BONES 256

cbuffer Object : register(b0)
{
    float4x4 gMVP;        // world * view * proj (applied AFTER skinning)
    float4x4 gModel;      // world placement, for lighting in world space
    float4   gBaseColor;
};

cbuffer Frame : register(b1)
{
    float4 gLightDir;     // .xyz direction the light travels
    float4 gCameraPos;
};

cbuffer Bones : register(b2)
{
    float4x4 gBones[MAX_BONES];
};

struct VSInput {
    float3 Position : POSITION;
    float3 Normal   : NORMAL;
    float2 TexCoord : TEXCOORD;
    float4 Tangent  : TANGENT;
    uint4  Joints   : BLENDINDICES;
    float4 Weights  : BLENDWEIGHT;
};
struct VSOutput {
    float4 Pos      : SV_POSITION;
    float3 WorldPos : TEXCOORD1;
    float3 Normal   : NORMAL;
    float2 Uv       : TEXCOORD0;
};

VSOutput VSMain(VSInput input)
{
    float3 skinnedPos = 0;
    float3 skinnedNrm = 0;
    [unroll]
    for (int k = 0; k < 4; ++k) {
        const float4x4 bone = gBones[input.Joints[k]];
        const float    w    = input.Weights[k];
        skinnedPos += mul(bone, float4(input.Position, 1.0)).xyz * w;
        skinnedNrm += mul((float3x3)bone, input.Normal) * w;
    }

    VSOutput o;
    o.Pos      = mul(gMVP,   float4(skinnedPos, 1.0));
    o.WorldPos = mul(gModel, float4(skinnedPos, 1.0)).xyz;
    o.Normal   = mul((float3x3)gModel, skinnedNrm);
    o.Uv       = input.TexCoord;
    return o;
}

float4 PSMain(VSOutput i) : SV_TARGET
{
    const float3 N = normalize(i.Normal);
    const float3 L = normalize(-gLightDir.xyz);
    const float3 V = normalize(gCameraPos.xyz - i.WorldPos);
    const float3 H = normalize(L + V);

    const float ndl  = saturate(dot(N, L));
    const float back = saturate(dot(N, -L)) * 0.15;          // soft fill light
    const float spec = pow(saturate(dot(N, H)), 32.0) * 0.25;

    const float3 albedo = gBaseColor.rgb;
    float3 color = albedo * (0.18 + ndl * 0.9 + back) + spec * ndl;
    return float4(pow(color, 1.0 / 2.2), 1.0);
}
