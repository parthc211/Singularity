// Bloom geometry pass: render objects into an HDR (RGBA16F) target. Colours can
// exceed 1.0 (emissive) so the bright-pass has something to extract.
cbuffer Object : register(b0)
{
    float4x4 gMVP;
    float4x4 gModel;
    float4   gColor; // HDR; bright objects use values > 1
};

struct VSInput  { float3 Position : POSITION; float3 Normal : NORMAL; float2 TexCoord : TEXCOORD; };
struct VSOutput { float4 Pos : SV_POSITION; float3 WorldNormal : NORMAL; };

VSOutput VSMain(VSInput i)
{
    VSOutput o;
    o.Pos         = mul(gMVP, float4(i.Position, 1.0));
    o.WorldNormal = mul((float3x3)gModel, i.Normal);
    return o;
}

float4 PSMain(VSOutput i) : SV_TARGET
{
    float3 n   = normalize(i.WorldNormal);
    float  ndl = saturate(dot(n, normalize(float3(0.3, 0.8, 0.4))));
    // Simple shading so faces read; emissive magnitude (in gColor) survives to bloom.
    return float4(gColor.rgb * (0.4 + 0.6 * ndl), 1.0);
}
