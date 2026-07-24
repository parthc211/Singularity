// Cascaded-shadow main pass: pick a cascade per pixel by view distance, project
// the pixel into that cascade's light space, and PCF-sample the matching slice
// of the shadow-map array.
#define MAX_CASCADES 4

cbuffer LitObject : register(b0)
{
    float4x4 gMVP;
    float4x4 gModel;
    float4   gBaseColor;
};

cbuffer LitFrame : register(b1)
{
    float4x4 gCascadeVP[MAX_CASCADES]; // world -> each cascade's light clip
    float4   gCascadeSplits;           // far view-distance of cascades 0..3
    float4   gLightDir;
    float4   gCameraPos;
    float    gBias;
    float    gTexelSize;
    int      gCascadeCount;
    int      gDebugTint;               // 1 = tint pixels by cascade
};

Texture2DArray         gShadow        : register(t0);
SamplerComparisonState gShadowSampler : register(s0);

struct VSInput  { float3 Position : POSITION; float3 Normal : NORMAL; float2 TexCoord : TEXCOORD; };
struct VSOutput { float4 Pos : SV_POSITION; float3 WorldNormal : NORMAL; float3 WorldPos : POSITION; };

VSOutput VSMain(VSInput i) {
    VSOutput o;
    o.Pos         = mul(gMVP,   float4(i.Position, 1.0));
    o.WorldNormal = mul((float3x3)gModel, i.Normal);
    o.WorldPos    = mul(gModel, float4(i.Position, 1.0)).xyz;
    return o;
}

float4 PSMain(VSOutput i) : SV_TARGET {
    // Choose the tightest cascade whose far distance still contains this pixel.
    float viewDist = distance(gCameraPos.xyz, i.WorldPos);
    int cascade = gCascadeCount - 1;
    [unroll] for (int c = 0; c < MAX_CASCADES; ++c)
        if (c < gCascadeCount && viewDist <= gCascadeSplits[c]) { cascade = c; break; }

    float4 lc  = mul(gCascadeVP[cascade], float4(i.WorldPos, 1.0));
    float3 ndc = lc.xyz / lc.w;
    float2 uv  = ndc.xy * float2(0.5, -0.5) + 0.5;
    float  ref = ndc.z - gBias;

    float shadow = 1.0;
    if (ndc.z <= 1.0) {
        float sum = 0.0; int cnt = 0;
        [unroll] for (int y = -1; y <= 1; ++y)
            for (int x = -1; x <= 1; ++x) {
                sum += gShadow.SampleCmpLevelZero(gShadowSampler,
                          float3(uv + float2(x, y) * gTexelSize, cascade), ref);
                ++cnt;
            }
        shadow = sum / cnt;
    }

    float3 N = normalize(i.WorldNormal);
    float3 L = normalize(-gLightDir.xyz);
    float  ndl = saturate(dot(N, L));
    float3 albedo = gBaseColor.rgb;
    float3 col = albedo * (0.25 + 0.75 * ndl * shadow);

    if (gDebugTint) {
        const float3 tint[4] = { float3(1, 0.5, 0.5), float3(0.5, 1, 0.5),
                                 float3(0.5, 0.6, 1), float3(1, 1, 0.5) };
        col *= tint[cascade];
    }
    return float4(col, 1.0);
}
