// Texture + normal mapping + parallax occlusion mapping.
//
// Albedo samples through an sRGB SRV (hardware-decoded to linear); per-pixel
// normals come from a tangent-space normal map whose ALPHA channel carries the
// surface height (1 = top, 0 = deepest). POM ray-marches that height field
// along the tangent-space view ray so brick edges actually occlude the mortar
// behind them — real depth from a flat quad. Lighting runs in linear light and
// gamma-encodes at the end (the swap chain is plain UNORM).

cbuffer Object : register(b0)
{
    float4x4 gMVP;
    float4x4 gModel;
    float4   gUvTiling;    // .xy = texcoord multiplier
    float4   gMaterial;    // x = roughness multiplier, y = metallic override
                           //     (< 0: take metalness from the ORM map)
};

cbuffer Frame : register(b1)
{
    float4 gLightDir;        // .xyz direction the light travels
    float4 gCameraPos;       // .xyz world-space eye
    float  gNormalStrength;  // scales the sampled XY perturbation
    int    gUseNormalMap;    // A/B toggle
    int    gFlipGreen;       // OpenGL-style maps store +Y up; DirectX wants -Y
    int    gViewMode;        // 0 lit, 1 albedo, 2 geometric N, 3 mapped N, 4 height
    int    gGammaCorrect;    // A/B: encode the linear result for the UNORM target
    float  gHeightScale;     // parallax depth in UV units (~0.03-0.10)
    int    gUsePom;          // A/B toggle
    int    gPomShadow;       // contact-shadow ray toward the light
    int    gPomSteps;        // max ray-march layers (min is steps/4 head-on)
    float  gRoughnessScale;  // global multiplier on sampled roughness
    int    gUseMaterialMap;  // A/B: ORM map vs constant material
    float  _pad;
};

Texture2D    gAlbedoMap : register(t0);   // R8G8B8A8_UNORM_SRGB — color data
Texture2D    gNormalMap : register(t1);   // R8G8B8A8_UNORM — xyz normal, w height
Texture2D    gOrmMap    : register(t2);   // R8G8B8A8_UNORM — R=AO, G=roughness, B=metalness
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

// Height-field DEPTH at uv: 0 at the surface top, 1 at the deepest point.
// SampleGrad keeps mip selection stable inside the divergent march loop.
float SampleDepth(float2 uv, float2 dx, float2 dy)
{
    return 1.0 - gNormalMap.SampleGrad(gSampler, uv, dx, dy).a;
}

// Classic layered POM: step the view ray down through the height field until
// it dips below the surface, then intersect the last two samples linearly.
// viewTS = tangent-space direction from surface point toward the EYE.
// Also returns the intersection depth for the shadow ray.
float2 ParallaxUV(float2 uv, float3 viewTS, float2 dx, float2 dy, out float hitDepth)
{
    // More layers at grazing angles, where parallax stretches furthest.
    const float layers     = lerp(float(gPomSteps), float(gPomSteps) * 0.25, saturate(viewTS.z));
    const float layerDepth = 1.0 / layers;
    // Total UV shift if the ray traversed the full depth range. The max() on
    // viewTS.z caps the shift at silhouettes (standard offset limiting).
    const float2 shift   = viewTS.xy / max(viewTS.z, 0.15) * gHeightScale;
    const float2 deltaUv = shift / layers;

    float2 curUv    = uv;
    float  rayDepth = 0.0;
    float  h        = SampleDepth(curUv, dx, dy);
    [loop]
    while (rayDepth < h) {
        curUv    -= deltaUv;         // march toward where the eye is looking
        rayDepth += layerDepth;
        h         = SampleDepth(curUv, dx, dy);
    }

    // Linear intersection between the last two layers.
    const float2 prevUv      = curUv + deltaUv;
    const float  afterDist   = h - rayDepth;                                   // <= 0
    const float  beforeDist  = SampleDepth(prevUv, dx, dy) - (rayDepth - layerDepth); // >= 0
    const float  w           = beforeDist / max(beforeDist - afterDist, 1e-5);
    hitDepth = rayDepth - (1.0 - w) * layerDepth;
    return lerp(prevUv, curUv, w);
}

// Contact shadow: march from the displaced point toward the light; if the
// height field rises above the ray, the point is occluded. Soft factor from
// the deepest penetration (percentage-closer style).
float ParallaxShadow(float2 uv, float startDepth, float3 lightTS, float2 dx, float2 dy)
{
    if (lightTS.z <= 0.01) return 0.0;   // light below the surface plane

    const float layers     = float(gPomSteps) * 0.5;
    const float layerDepth = startDepth / layers;   // climb from hit depth to 0
    if (layerDepth <= 0.0) return 1.0;
    const float2 deltaUv = lightTS.xy / max(lightTS.z, 0.15) * gHeightScale / layers;

    float2 curUv       = uv;
    float  rayDepth    = startDepth;
    float  penetration = 0.0;
    [loop]
    for (float i = 0.0; i < layers; i += 1.0) {
        curUv    += deltaUv;         // toward the light
        rayDepth -= layerDepth;
        const float h = SampleDepth(curUv, dx, dy);
        penetration = max(penetration, rayDepth - h);   // how far above the ray the surface pokes
        if (rayDepth <= 0.0) break;
    }
    // penetration 0 -> fully lit; deeper occluders darken faster.
    return saturate(1.0 - penetration * 12.0);
}

float4 PSMain(VSOutput i) : SV_TARGET
{
    const float3 Ngeo = normalize(i.Normal);
    const float3 T    = normalize(i.Tangent.xyz - Ngeo * dot(Ngeo, i.Tangent.xyz));
    const float3 B    = cross(Ngeo, T) * i.Tangent.w;
    const float3 V    = normalize(gCameraPos.xyz - i.WorldPos);
    const float3 L    = normalize(-gLightDir.xyz);

    // World -> tangent space (rows of the TBN basis).
    const float3 viewTS  = float3(dot(V, T), dot(V, B), dot(V, Ngeo));
    const float3 lightTS = float3(dot(L, T), dot(L, B), dot(L, Ngeo));

    const float2 dx = ddx(i.Uv), dy = ddy(i.Uv);

    float2 uv       = i.Uv;
    float  hitDepth = 0.0;
    if (gUsePom != 0 && viewTS.z > 0.0)
        uv = ParallaxUV(i.Uv, viewTS, dx, dy, hitDepth);

    float3 N = Ngeo;
    if (gUseNormalMap != 0) {
        float3 nTex = gNormalMap.SampleGrad(gSampler, uv, dx, dy).xyz * 2.0 - 1.0;
        if (gFlipGreen != 0) nTex.y = -nTex.y;
        nTex.xy *= gNormalStrength;
        N = normalize(nTex.x * T + nTex.y * B + nTex.z * Ngeo);
    }

    // sRGB SRV: the sample arrives already linear.
    const float3 albedo = gAlbedoMap.SampleGrad(gSampler, uv, dx, dy).rgb;

    if (gViewMode == 1) return float4(albedo, 1.0);
    if (gViewMode == 2) return float4(Ngeo * 0.5 + 0.5, 1.0);
    if (gViewMode == 3) return float4(N * 0.5 + 0.5, 1.0);
    if (gViewMode == 4) {
        const float h = gNormalMap.SampleGrad(gSampler, uv, dx, dy).a;
        return float4(h, h, h, 1.0);
    }

    // ORM material (glTF packing): R = ambient occlusion, G = roughness,
    // B = metalness. The A/B toggle swaps in a neutral constant material.
    float3 orm = gUseMaterialMap != 0
               ? gOrmMap.SampleGrad(gSampler, uv, dx, dy).rgb
               : float3(1.0, 0.6, 0.0);
    const float ao        = orm.r;
    const float roughness = clamp(orm.g * gRoughnessScale * gMaterial.x, 0.04, 1.0);
    const float metallic  = gMaterial.y >= 0.0 ? saturate(gMaterial.y) : orm.b;

    if (gViewMode == 5) return float4(orm, 1.0);

    float shadow = 1.0;
    if (gUsePom != 0 && gPomShadow != 0)
        shadow = ParallaxShadow(uv, hitDepth, lightTS, dx, dy);

    // Cook-Torrance GGX in linear light. F0: 4% dielectric, albedo for metals.
    const float3 H   = normalize(L + V);
    const float  NoL = saturate(dot(N, L));
    const float  NoV = max(dot(N, V), 1e-4);
    const float  NoH = saturate(dot(N, H));
    const float  VoH = saturate(dot(V, H));

    const float3 F0 = lerp(0.04, albedo, metallic);
    const float3 F  = F0 + (1.0 - F0) * pow(1.0 - VoH, 5.0);      // Fresnel-Schlick

    const float a  = roughness * roughness;                        // GGX/Trowbridge-Reitz
    const float a2 = a * a;
    const float dTerm = NoH * NoH * (a2 - 1.0) + 1.0;
    const float D  = a2 / max(3.14159265 * dTerm * dTerm, 1e-6);

    const float k  = (roughness + 1.0) * (roughness + 1.0) / 8.0;  // Smith-Schlick
    const float G  = (NoV / (NoV * (1.0 - k) + k)) * (NoL / (NoL * (1.0 - k) + k));

    const float3 specular = D * G * F / max(4.0 * NoV * NoL, 1e-4);
    const float3 diffuse  = (1.0 - F) * (1.0 - metallic) * albedo / 3.14159265;

    // Directional light with pi-compensated intensity, plus a small ambient:
    // diffuse ambient for dielectrics and a crude F0 ambient so metals don't
    // go black without image-based lighting (a documented placeholder).
    const float3 lightColor = 3.1;
    const float3 direct  = (diffuse + specular) * NoL * shadow * lightColor;
    const float3 ambient = (0.055 * albedo * (1.0 - metallic) + F0 * 0.05) * ao;

    float3 color = ambient + direct;
    if (gGammaCorrect != 0)
        color = pow(color, 1.0 / 2.2);
    return float4(color, 1.0);
}
