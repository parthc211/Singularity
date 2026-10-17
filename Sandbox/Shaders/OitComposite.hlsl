// OitComposite.hlsl — resolve weighted-blended OIT over the opaque back buffer.
//
// averageColour = accum.rgb / accum.a  (the weighted average of all the
// transparent layers at this pixel), and revealage is how much background still
// shows through. Output (averageColour, revealage) is blended onto the back
// buffer with (INV_SRC_ALPHA, SRC_ALPHA), giving
//   final = averageColour * (1 - revealage) + opaque * revealage.

Texture2D    gAccum     : register(t0);   // RGBA16F
Texture2D    gRevealage : register(t1);   // R16F
SamplerState gSamp      : register(s0);

struct VSOut { float4 Pos : SV_POSITION; float2 Uv : TEXCOORD0; };

VSOut VSMain(uint vid : SV_VertexID)
{
    float2 uv = float2((vid << 1) & 2, vid & 2);
    VSOut o; o.Uv = uv; o.Pos = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    o.Pos.y = -o.Pos.y;
    return o;
}

float4 PSMain(VSOut i) : SV_TARGET
{
    const float4 accum  = gAccum.SampleLevel(gSamp, i.Uv, 0);
    const float  reveal = gRevealage.SampleLevel(gSamp, i.Uv, 0).r;
    const float3 avg    = accum.rgb / max(accum.a, 1e-5);
    return float4(avg, reveal);   // blended: avg*(1-reveal) + opaque*reveal
}
