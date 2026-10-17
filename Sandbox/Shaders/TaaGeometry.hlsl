// TaaGeometry.hlsl — the geometry pass for temporal anti-aliasing. It writes two
// targets: shaded colour, and a screen-space MOTION VECTOR (where did this pixel
// sit last frame). The rasterization position is JITTERED sub-pixel each frame so
// that, accumulated over frames, many sample positions are covered (the "free"
// supersampling TAA buys). Crucially the motion vector is computed from the
// UN-jittered current/previous clip positions, so reprojection tracks true
// geometric motion rather than the jitter wobble.

cbuffer Object : register(b0)
{
    float4x4 gJitteredMVP;    // model * view * proj * jitter  (for SV_Position)
    float4x4 gUnjitteredMVP;  // model * view * proj           (for motion)
    float4x4 gPrevMVP;        // prevModel * prevView * prevProj
    float4x4 gModel;
    float4   gColor;
}

struct VSOut {
    float4 Pos      : SV_POSITION;
    float4 CurrClip : TEXCOORD0;   // unjittered
    float4 PrevClip : TEXCOORD1;
    float3 Normal   : NORMAL;
};

struct PSOut {
    float4 Color    : SV_TARGET0;
    float2 Velocity : SV_TARGET1;  // RG16F: historyUV = uv + velocity
};

VSOut VSMain(float3 pos : POSITION, float3 nrm : NORMAL)
{
    VSOut o;
    o.Pos      = mul(gJitteredMVP,   float4(pos, 1.0));
    o.CurrClip = mul(gUnjitteredMVP, float4(pos, 1.0));
    o.PrevClip = mul(gPrevMVP,       float4(pos, 1.0));
    o.Normal   = mul((float3x3)gModel, nrm);
    return o;
}

float2 ClipToUv(float4 clip)
{
    float2 ndc = clip.xy / clip.w;
    return float2(ndc.x * 0.5 + 0.5, -ndc.y * 0.5 + 0.5);
}

PSOut PSMain(VSOut i)
{
    const float3 N = normalize(i.Normal);
    const float3 L = normalize(float3(0.4, 0.8, -0.4));
    const float  ndl = saturate(dot(N, L));

    PSOut o;
    o.Color    = float4(gColor.rgb * (0.12 + 0.88 * ndl), 1.0);
    o.Velocity = ClipToUv(i.PrevClip) - ClipToUv(i.CurrClip);  // curr -> prev
    return o;
}
