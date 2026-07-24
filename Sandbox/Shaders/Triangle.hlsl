// Per-object constants. Layout must match SGE::ObjectConstants (RenderSystem.cpp).
// One of these is bound per draw via SetGraphicsRootConstantBufferView(b0).
cbuffer ObjectConstants : register(b0)
{
    float4x4 gMVP;        // model * view * proj (engine uploads row-major; HLSL
                          // reads column-major == implicit transpose)
    float4x4 gModel;      // model matrix, for transforming normals to world
    float4   gBaseColor;  // tint from MaterialComponent (white if none)
};

struct VSInput
{
    float3 Position : POSITION;
    float3 Normal   : NORMAL;
    float2 TexCoord : TEXCOORD;
};

struct VSOutput
{
    float4 Position : SV_POSITION;
    float3 Normal   : NORMAL;
};

VSOutput VSMain(VSInput input)
{
    VSOutput o;
    // mul(M, v): matches the engine's MVP convention (do not flip to mul(v, M)).
    o.Position = mul(gMVP, float4(input.Position, 1.0));
    // Same convention applied to the model matrix rotates the normal into world
    // space. (Assumes uniform scale; non-uniform scale needs the inverse-transpose.)
    o.Normal = mul((float3x3)gModel, input.Normal);
    return o;
}

float4 PSMain(VSOutput input) : SV_TARGET
{
    float3 n = normalize(input.Normal) * 0.5 + 0.5; // map [-1,1] -> [0,1]
    return float4(n * gBaseColor.rgb, 1.0);
}
