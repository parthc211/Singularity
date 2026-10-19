// RtBlit.hlsl — copy the ray-tracing output texture to the back buffer with a
// fullscreen triangle (so ImGui can still composite over the back buffer after).

Texture2D    gTex  : register(t0);
SamplerState gSamp : register(s0);

struct VSOut { float4 Pos : SV_POSITION; float2 Uv : TEXCOORD0; };

VSOut VSMain(uint vid : SV_VertexID)
{
    float2 uv = float2((vid << 1) & 2, vid & 2);
    VSOut o; o.Uv = uv; o.Pos = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    o.Pos.y = -o.Pos.y;
    return o;
}

float4 PSMain(VSOut i) : SV_TARGET { return gTex.SampleLevel(gSamp, i.Uv, 0); }
