// Texture + normal mapping: albedo sampled from an sRGB SRV (the hardware
// decodes to linear on read), and per-pixel normals from a tangent-space
// normal map. The TBN basis comes from the mesh's per-vertex tangents
// (T = +U direction, w = handedness): B = cross(N, T) * w, and the sampled
// normal (unpacked from [0,1] to [-1,1]) rotates from tangent space to world.
// Lighting runs in linear light; the final color is gamma-encoded because the
// swap chain is plain UNORM (no automatic sRGB conversion on write).

cbuffer Object : register(b0)
{
    float4x4 gMVP;
    float4x4 gModel;
    float4   gUvTiling;   // .xy = texcoord multiplier
};

cbuffer Frame : register(b1)
{
    float4 gLightDir;        // .xyz direction the light travels
    float4 gCameraPos;       // .xyz world-space eye
    float  gNormalStrength;  // scales the sampled XY perturbation
    int    gUseNormalMap;    // A/B toggle
    int    gFlipGreen;       // OpenGL-style maps store +Y up; DirectX wants -Y
    int    gViewMode;        // 0 lit, 1 albedo, 2 geometric N, 3 mapped N
    int    gGammaCorrect;    // A/B: encode the linear result for the UNORM target
    float3 _pad;
};

Texture2D    gAlbedoMap : register(t0);   // R8G8B8A8_UNORM_SRGB — color data
Texture2D    gNormalMap : register(t1);   // R8G8B8A8_UNORM      — vector data
SamplerState gSampler   : register(s0);   // anisotropic, wrap

struct VSInput {
    float3 Position : POSITION;
    float3 Normal   : NORMAL;
    float2 TexCoord : TEXCOORD;
    float4 Tangent  : TANGENT;   // xyz tangent, w handedness
};
struct VSOutput {
    float4 Pos      : SV_POSITION;
    float3 WorldPos : TEXCOORD1;
    float3 Normal   : NORMAL;    // world-space geometric normal
    float4 Tangent  : TANGENT;   // world-space tangent + handedness
    float2 Uv       : TEXCOORD0;
};

VSOutput VSMain(VSInput input)
{
    VSOutput o;
    o.Pos       = mul(gMVP, float4(input.Position, 1.0));
    o.WorldPos  = mul(gModel, float4(input.Position, 1.0)).xyz;
    o.Normal    = mul((float3x3)gModel, input.Normal);
    o.Tangent   = float4(mul((float3x3)gModel, input.Tangent.xyz), input.Tangent.w);
    o.Uv        = input.TexCoord * gUvTiling.xy;
    return o;
}

float4 PSMain(VSOutput i) : SV_TARGET
{
    float3 N = normalize(i.Normal);

    if (gUseNormalMap != 0) {
        // Re-orthonormalize after interpolation, then build the TBN frame.
        float3 T = normalize(i.Tangent.xyz - N * dot(N, i.Tangent.xyz));
        float3 B = cross(N, T) * i.Tangent.w;

        float3 nTex = gNormalMap.Sample(gSampler, i.Uv).xyz * 2.0 - 1.0;
        if (gFlipGreen != 0) nTex.y = -nTex.y;
        nTex.xy *= gNormalStrength;
        N = normalize(nTex.x * T + nTex.y * B + nTex.z * normalize(i.Normal));
    }

    // sRGB SRV: the sample arrives already linear.
    float3 albedo = gAlbedoMap.Sample(gSampler, i.Uv).rgb;

    if (gViewMode == 1) return float4(albedo, 1.0);
    if (gViewMode == 2) return float4(normalize(i.Normal) * 0.5 + 0.5, 1.0);
    if (gViewMode == 3) return float4(N * 0.5 + 0.5, 1.0);

    // Blinn-Phong in linear light.
    float3 L = normalize(-gLightDir.xyz);
    float3 V = normalize(gCameraPos.xyz - i.WorldPos);
    float3 H = normalize(L + V);

    float  ndl      = saturate(dot(N, L));
    float  spec     = pow(saturate(dot(N, H)), 48.0) * 0.35;
    float3 ambient  = 0.06 * albedo;
    float3 color    = ambient + albedo * ndl + spec * ndl;

    if (gGammaCorrect != 0)
        color = pow(color, 1.0 / 2.2);
    return float4(color, 1.0);
}
