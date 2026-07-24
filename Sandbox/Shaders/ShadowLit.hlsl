// Main pass: shade with a directional light and test each pixel against the
// shadow map. A pixel is in shadow if, in light space, something is closer to the
// light than it is. PCF (percentage-closer filtering) samples the map at several
// nearby texels through a hardware comparison sampler for soft edges.

cbuffer LitObject : register(b0)
{
    float4x4 gMVP;       // model * view * proj  (camera clip)
    float4x4 gModel;     // for world-space normal
    float4x4 gLightMVP;  // model * lightView * lightProj (light clip)
    float4   gBaseColor;
};

cbuffer LitFrame : register(b1)
{
    float4 gLightDir;   // .xyz direction the light travels
    float4 gCameraPos;
    float  gBias;       // depth bias to fight shadow acne
    float  gTexelSize;  // 1 / shadowMapResolution (PCF tap spacing)
    int    gPcfRadius;  // PCF kernel half-width (0 = 1 tap)
    float  _pad;
};

Texture2D              gShadowMap     : register(t0);
SamplerComparisonState gShadowSampler : register(s0); // compares ref vs stored depth

struct VSInput {
    float3 Position : POSITION;
    float3 Normal   : NORMAL;
    float2 TexCoord : TEXCOORD;
};
struct VSOutput {
    float4 Pos         : SV_POSITION;
    float3 WorldNormal : NORMAL;
    float4 LightClip   : TEXCOORD0; // position in light clip space
};

VSOutput VSMain(VSInput input) {
    VSOutput o;
    o.Pos         = mul(gMVP,      float4(input.Position, 1.0));
    o.WorldNormal = mul((float3x3)gModel, input.Normal);
    o.LightClip   = mul(gLightMVP, float4(input.Position, 1.0));
    return o;
}

// Returns 1 = fully lit, 0 = fully shadowed (averaged over the PCF kernel).
float SampleShadow(float4 lightClip) {
    float3 ndc = lightClip.xyz / lightClip.w;
    if (ndc.z > 1.0) return 1.0;                 // beyond the light's far plane
    float2 uv  = ndc.xy * float2(0.5, -0.5) + 0.5; // clip [-1,1] -> UV [0,1] (flip Y)
    float  ref = ndc.z - gBias;

    float sum = 0.0; int count = 0;
    [loop] for (int y = -gPcfRadius; y <= gPcfRadius; ++y)
        for (int x = -gPcfRadius; x <= gPcfRadius; ++x) {
            // SampleCmpLevelZero: hardware compares 'ref' to the stored depth and
            // (with a comparison filter) returns the lit fraction across the tap.
            sum += gShadowMap.SampleCmpLevelZero(gShadowSampler, uv + float2(x, y) * gTexelSize, ref);
            ++count;
        }
    return sum / count;
}

float4 PSMain(VSOutput i) : SV_TARGET {
    float3 N = normalize(i.WorldNormal);
    float3 L = normalize(-gLightDir.xyz);          // toward the light
    float  ndl = saturate(dot(N, L));
    float  shadow = SampleShadow(i.LightClip);

    float3 albedo  = gBaseColor.rgb;
    float3 ambient = 0.25 * albedo;
    float3 diffuse = albedo * ndl * shadow;        // direct light is gated by shadow
    return float4(ambient + diffuse, 1.0);
}
