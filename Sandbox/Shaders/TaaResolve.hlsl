// TaaResolve.hlsl — the temporal resolve. For each pixel: take the current
// (jittered) colour, reproject last frame's resolved history along the motion
// vector, and blend them (history-heavy). History is clamped to the AABB of the
// current 3x3 colour neighbourhood — the standard ghosting fix: when the history
// colour falls outside what's plausible here now (a disocclusion, a fast edge),
// it is pulled back into range instead of smearing.
//
// Outputs to two targets at once: the new history (linear, fed back next frame)
// and the back buffer (gamma-encoded). With TAA off it passes the current colour
// straight through so the A/B toggle shows raw aliasing.

cbuffer Params : register(b0)
{
    float2 gTexel;        // 1/width, 1/height
    int    gHistoryValid; // 0 on the first frame / after resize
    int    gTaaEnabled;
    float  gBlend;        // current-frame weight (~0.1 = 90% history)
}

Texture2D    gColor    : register(t0);
Texture2D    gVelocity : register(t1);
Texture2D    gHistory  : register(t2);
SamplerState gSamp     : register(s0);

struct VSOut { float4 Pos : SV_POSITION; float2 Uv : TEXCOORD0; };
struct PSOut { float4 History : SV_TARGET0; float4 Back : SV_TARGET1; };

VSOut VSMain(uint vid : SV_VertexID)
{
    float2 uv = float2((vid << 1) & 2, vid & 2);
    VSOut o; o.Uv = uv; o.Pos = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    o.Pos.y = -o.Pos.y;
    return o;
}

PSOut PSMain(VSOut i)
{
    const float2 uv = i.Uv;
    float3 cur = gColor.SampleLevel(gSamp, uv, 0).rgb;

    PSOut o;
    if (gTaaEnabled == 0) {
        o.History = float4(cur, 1.0);
        o.Back    = float4(pow(saturate(cur), 1.0 / 2.2), 1.0);
        return o;
    }

    // Reproject history along the motion vector.
    const float2 mv     = gVelocity.SampleLevel(gSamp, uv, 0).xy;
    const float2 histUv = uv + mv;
    float3 hist = gHistory.SampleLevel(gSamp, histUv, 0).rgb;

    // 3x3 neighbourhood AABB of the current frame, used to clamp history.
    float3 nmin = cur, nmax = cur;
    [unroll] for (int y = -1; y <= 1; ++y)
        [unroll] for (int x = -1; x <= 1; ++x) {
            float3 c = gColor.SampleLevel(gSamp, uv + float2(x, y) * gTexel, 0).rgb;
            nmin = min(nmin, c); nmax = max(nmax, c);
        }
    hist = clamp(hist, nmin, nmax);

    const bool onScreen = all(histUv >= 0.0) && all(histUv <= 1.0);
    const float a = (gHistoryValid != 0 && onScreen) ? gBlend : 1.0;
    float3 outc = lerp(hist, cur, a);

    o.History = float4(outc, 1.0);
    o.Back    = float4(pow(saturate(outc), 1.0 / 2.2), 1.0);
    return o;
}
