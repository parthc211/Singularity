// PostBright.hlsl — bloom bright-pass: keep only the part of the HDR image above
// a luminance threshold, with a soft knee so the transition isn't a hard edge.
// Fullscreen triangle; output feeds the separable blur.

cbuffer Params : register(b0)
{
    float gThreshold;   // luminance above which pixels bloom
    float gKnee;        // soft-knee width
    float2 _pad;
}

Texture2D    gScene : register(t0);
SamplerState gSamp  : register(s0);

struct VSOut { float4 Pos : SV_POSITION; float2 Uv : TEXCOORD0; };

VSOut VSMain(uint vid : SV_VertexID)
{
    float2 uv = float2((vid << 1) & 2, vid & 2);
    VSOut o; o.Uv = uv; o.Pos = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    o.Pos.y = -o.Pos.y;   // flip so uv.y=0 is the top
    return o;
}

float4 PSMain(VSOut i) : SV_TARGET
{
    float3 c = gScene.SampleLevel(gSamp, i.Uv, 0).rgb;
    const float lum = dot(c, float3(0.2126, 0.7152, 0.0722));
    // Soft-knee curve (Karis): smooth ramp from threshold-knee to threshold+knee.
    const float soft = clamp(lum - gThreshold + gKnee, 0.0, 2.0 * gKnee);
    const float contrib = max(soft * soft / (4.0 * gKnee + 1e-4), lum - gThreshold) / max(lum, 1e-4);
    return float4(c * max(contrib, 0.0), 1.0);
}
