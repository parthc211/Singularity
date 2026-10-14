// Bindless.hlsl — one root signature, many textures (SM 6.6).
//
// There is NO texture in the root signature and NO per-draw descriptor table.
// Each object passes only an integer (gTexIndex) in its constant buffer; the
// pixel shader pulls the texture straight out of the bound descriptor heap with
// ResourceDescriptorHeap[gTexIndex]. The heap is bound once for the whole grid.
// Requires HLSL 2021 (-HV 2021) and a vs_6_6 / ps_6_6 target.

cbuffer Object : register(b0)
{
    float4x4 gMVP;
    float4x4 gModel;
    uint     gTexIndex;   // index INTO the bound CBV/SRV/UAV heap
    float3   _pad;
}

SamplerState gSampler : register(s0);

struct VSOut {
    float4 Pos    : SV_POSITION;
    float3 Normal : NORMAL;
    float2 Uv     : TEXCOORD0;
};

VSOut VSMain(float3 pos : POSITION, float3 nrm : NORMAL, float2 uv : TEXCOORD)
{
    VSOut o;
    o.Pos    = mul(gMVP, float4(pos, 1.0));
    o.Normal = mul((float3x3)gModel, nrm);
    o.Uv     = uv;
    return o;
}

float4 PSMain(VSOut i) : SV_TARGET
{
    // The bindless fetch: an integer indexes the whole heap, no binding state.
    Texture2D tex = ResourceDescriptorHeap[gTexIndex];
    float3 albedo = tex.Sample(gSampler, i.Uv).rgb;

    // A little directional shading so the cubes read as 3D.
    const float3 L   = normalize(float3(0.4, 0.8, -0.4));
    const float  ndl = saturate(dot(normalize(i.Normal), L));
    float3 color = albedo * (0.25 + 0.75 * ndl);
    return float4(color, 1.0);
}
