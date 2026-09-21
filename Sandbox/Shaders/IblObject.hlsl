// IblObject.hlsl — PBR shading with image-based lighting (the payoff of the bake).
//
// Ambient is no longer the flat F0 placeholder: diffuse comes from the irradiance
// cube (irradiance * albedo), specular from the prefiltered env cube indexed by
// roughness, weighted by the split-sum BRDF LUT — so metals reflect their
// surroundings and dielectrics pick up sky/ground colour. An optional analytic
// sun (same Cook-Torrance as the normal-map demo) adds a crisp direct highlight.

cbuffer Object : register(b0)
{
    float4x4 gMVP;
    float4x4 gModel;
    float4   gAlbedo;     // rgb base colour
    float4   gMaterial;   // x roughness, y metallic
}

cbuffer Frame : register(b1)
{
    float4 gCamPos;
    float4 gSunDir;        // direction TO the sun
    float  gSunIntensity;
    int    gUseDirect;     // add the analytic sun highlight
    int    gDiffuseIbl;    // A/B toggles for teaching
    int    gSpecularIbl;
    float  gIblIntensity;
    float  gMaxMip;        // prefiltered cube's top mip index
    float  gExposure;
    int    gGamma;
}

TextureCube<float4> gIrradiance  : register(t0);
TextureCube<float4> gPrefiltered : register(t1);
Texture2D<float2>   gBrdfLut     : register(t2);
SamplerState        gSamp        : register(s0);

static const float PI = 3.14159265359;

struct VSOut {
    float4 Pos      : SV_POSITION;
    float3 WorldPos : TEXCOORD0;
    float3 Normal   : NORMAL;
};

VSOut VSMain(float3 pos : POSITION, float3 nrm : NORMAL)
{
    VSOut o;
    o.Pos      = mul(gMVP, float4(pos, 1.0));
    o.WorldPos = mul(gModel, float4(pos, 1.0)).xyz;
    o.Normal   = mul((float3x3)gModel, nrm);
    return o;
}

float3 FresnelSchlick(float cosT, float3 F0)          { return F0 + (1.0 - F0) * pow(1.0 - cosT, 5.0); }
float3 FresnelSchlickRough(float cosT, float3 F0, float r)
{
    return F0 + (max(1.0 - r, F0) - F0) * pow(1.0 - cosT, 5.0);
}

float4 PSMain(VSOut i) : SV_TARGET
{
    const float3 N   = normalize(i.Normal);
    const float3 V   = normalize(gCamPos.xyz - i.WorldPos);
    const float3 R   = reflect(-V, N);
    const float  NoV = max(dot(N, V), 1e-4);

    const float3 albedo    = gAlbedo.rgb;
    const float  roughness = clamp(gMaterial.x, 0.04, 1.0);
    const float  metallic  = saturate(gMaterial.y);
    const float3 F0        = lerp(0.04, albedo, metallic);

    // ---- Image-based lighting (split-sum) ----
    const float3 F  = FresnelSchlickRough(NoV, F0, roughness);
    const float3 kD = (1.0 - F) * (1.0 - metallic);

    float3 diffuse = 0.0;
    if (gDiffuseIbl != 0)
        diffuse = gIrradiance.SampleLevel(gSamp, N, 0).rgb * albedo;

    float3 specular = 0.0;
    if (gSpecularIbl != 0) {
        const float3 pre  = gPrefiltered.SampleLevel(gSamp, R, roughness * gMaxMip).rgb;
        const float2 brdf = gBrdfLut.SampleLevel(gSamp, float2(NoV, roughness), 0).rg;
        specular = pre * (F * brdf.x + brdf.y);
    }
    float3 color = (kD * diffuse + specular) * gIblIntensity;

    // ---- Optional analytic sun (direct Cook-Torrance GGX) ----
    if (gUseDirect != 0) {
        const float3 L   = normalize(gSunDir.xyz);
        const float3 H   = normalize(L + V);
        const float  NoL = saturate(dot(N, L));
        const float  NoH = saturate(dot(N, H));
        const float  VoH = saturate(dot(V, H));
        if (NoL > 0.0) {
            const float a = roughness * roughness, a2 = a * a;
            const float dT = NoH * NoH * (a2 - 1.0) + 1.0;
            const float D  = a2 / max(PI * dT * dT, 1e-6);
            const float k  = (roughness + 1.0) * (roughness + 1.0) / 8.0;
            const float G  = (NoV / (NoV * (1.0 - k) + k)) * (NoL / (NoL * (1.0 - k) + k));
            const float3 Fd = FresnelSchlick(VoH, F0);
            const float3 spec = D * G * Fd / max(4.0 * NoV * NoL, 1e-4);
            const float3 diff = (1.0 - Fd) * (1.0 - metallic) * albedo / PI;
            color += (diff + spec) * NoL * gSunIntensity;
        }
    }

    // Tonemap + gamma to match the skybox (UNORM back buffer).
    color *= gExposure;
    color = (color * (2.51 * color + 0.03)) / (color * (2.43 * color + 0.59) + 0.14);
    if (gGamma != 0) color = pow(saturate(color), 1.0 / 2.2);
    return float4(color, 1.0);
}
