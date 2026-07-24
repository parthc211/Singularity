// Composite: modulate the G-buffer albedo by the (blurred) ambient occlusion.
// Debug modes let you see AO applied, the raw AO, or albedo with no AO.
cbuffer CompositeCB : register(b0)
{
    int   gMode; // 0 = albedo * AO, 1 = AO only, 2 = albedo only
    float gAmbient;
    float2 _pad;
};

Texture2D    gAlbedo  : register(t0);
Texture2D    gAO      : register(t1);
SamplerState gSampler : register(s0);

struct VSOutput { float4 Pos : SV_POSITION; float2 UV : TEXCOORD0; };

VSOutput VSMain(uint id : SV_VertexID)
{
    VSOutput o;
    o.UV  = float2((id << 1) & 2, id & 2);
    o.Pos = float4(o.UV * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    return o;
}

float4 PSMain(VSOutput i) : SV_TARGET
{
    float3 albedo = gAlbedo.Sample(gSampler, i.UV).rgb;
    float  ao     = gAO.Sample(gSampler, i.UV).r;

    if (gMode == 1) return float4(ao.xxx, 1.0);          // raw AO (grayscale)
    if (gMode == 2) return float4(albedo, 1.0);          // albedo, no AO

    // A flat ambient term scaled by AO — isolates the AO contribution cleanly.
    return float4(albedo * (gAmbient + (1.0 - gAmbient) * ao), 1.0);
}
