// Deferred geometry pass: transform geometry and write surface attributes into
// the G-buffer (multiple render targets) — NO lighting here. The per-object
// cbuffer layout matches SGE::ObjectConstants (RenderSystem.cpp), so the same
// RenderSystem that feeds the forward shader feeds this one.
cbuffer ObjectConstants : register(b0)
{
    float4x4 gMVP;       // model * view * proj (engine uploads row-major; HLSL reads column-major)
    float4x4 gModel;     // model matrix (for world-space normal & position)
    float4   gBaseColor; // material tint
};

struct VSInput
{
    float3 Position : POSITION;
    float3 Normal   : NORMAL;
    float2 TexCoord : TEXCOORD;
};

struct VSOutput
{
    float4 Position    : SV_POSITION; // clip space
    float3 WorldNormal : NORMAL;
    float3 WorldPos    : POSITION;
};

VSOutput VSMain(VSInput input)
{
    VSOutput o;
    o.Position    = mul(gMVP, float4(input.Position, 1.0));        // mul(M,v) per engine convention
    o.WorldNormal = mul((float3x3)gModel, input.Normal);          // assumes ~uniform scale
    o.WorldPos    = mul(gModel, float4(input.Position, 1.0)).xyz; // model -> world
    return o;
}

// One PS output per G-buffer target (must match GBuffer::kFormats order).
struct PSOutput
{
    float4 Albedo   : SV_Target0; // RGBA8   : base colour
    float4 Normal   : SV_Target1; // RGBA16F : world-space normal (signed, stored directly)
    float4 Position : SV_Target2; // RGBA16F : world-space position
};

PSOutput PSMain(VSOutput input)
{
    PSOutput o;
    o.Albedo   = float4(gBaseColor.rgb, 1.0);
    o.Normal   = float4(normalize(input.WorldNormal), 0.0);
    o.Position = float4(input.WorldPos, 1.0);
    return o;
}
