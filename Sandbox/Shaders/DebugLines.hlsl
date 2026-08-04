// Colored line list for debug drawing (ground grid, skeleton overlay).
// One CBV: the full transform for this batch of lines.

cbuffer LineCB : register(b0)
{
    float4x4 gTransform;  // (model *) view * proj
};

struct VSInput {
    float3 Position : POSITION;
    float3 Color    : COLOR;
};
struct VSOutput {
    float4 Pos   : SV_POSITION;
    float3 Color : COLOR;
};

VSOutput VSMain(VSInput input)
{
    VSOutput o;
    o.Pos   = mul(gTransform, float4(input.Position, 1.0));
    o.Color = input.Color;
    return o;
}

float4 PSMain(VSOutput i) : SV_TARGET
{
    return float4(pow(i.Color, 1.0 / 2.2), 1.0);
}
