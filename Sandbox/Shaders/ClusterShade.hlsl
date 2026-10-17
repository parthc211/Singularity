// ClusterShade.hlsl — forward shading that loops only each pixel's froxel lights.
//
// From the pixel's screen position and view-space depth it reconstructs the
// froxel index (same exponential depth slicing as the AABB build), reads that
// froxel's (offset, count) from the light grid, and accumulates only those
// lights. A heatmap debug mode colours by light count so the culling is visible.

struct Light { float3 Pos; float Radius; float3 Color; float Intensity; };

cbuffer Object : register(b0)
{
    float4x4 gMVP;
    float4x4 gModel;
    float4   gAlbedo;
}

cbuffer Frame : register(b1)
{
    float4x4 gView;
    uint3    gGrid;
    uint     gMode;        // 0 = lit, 1 = light-count heatmap
    float    gNear;
    float    gFar;
    float2   gScreen;
    float3   gCamPos;
    float    gTileX;       // screen width / gGrid.x
    float    gTileY;       // screen height / gGrid.y
    float3   _pad;
}

StructuredBuffer<Light>  gLights    : register(t0);
StructuredBuffer<uint2>  gLightGrid : register(t1);
StructuredBuffer<uint>   gIndexList : register(t2);

struct VSOut {
    float4 Pos      : SV_POSITION;
    float3 WorldPos : TEXCOORD0;
    float3 Normal   : NORMAL;
};

VSOut VSMain(float3 pos : POSITION, float3 nrm : NORMAL)
{
    VSOut o;
    o.Pos      = mul(gMVP, float4(pos, 1.0));
    o.WorldPos = mul(gModel, float4(pos, 1.0)).xyz;
    o.Normal   = mul((float3x3)gModel, nrm);
    return o;
}

float3 Heatmap(float t)
{
    t = saturate(t);
    return float3(saturate(t * 1.5), saturate(1.5 - abs(t - 0.5) * 3.0), saturate(1.5 - t * 1.5));
}

float4 PSMain(VSOut i) : SV_TARGET
{
    const float viewZ = mul(gView, float4(i.WorldPos, 1.0)).z;

    const uint tileX = (uint)(i.Pos.x / gTileX);
    const uint tileY = (uint)(i.Pos.y / gTileY);
    const uint slice = (uint)(log(max(viewZ, gNear) / gNear) / log(gFar / gNear) * gGrid.z);
    const uint froxel = min(tileX, gGrid.x - 1)
                      + min(tileY, gGrid.y - 1) * gGrid.x
                      + min(slice, gGrid.z - 1) * gGrid.x * gGrid.y;

    const uint2 oc    = gLightGrid[froxel];
    const uint  off   = oc.x;
    const uint  count = oc.y;

    if (gMode == 1)
        return float4(Heatmap(float(count) / 32.0), 1.0);

    const float3 N = normalize(i.Normal);
    const float3 albedo = gAlbedo.rgb;
    float3 color = albedo * 0.03;   // ambient

    for (uint k = 0; k < count; ++k) {
        const Light L = gLights[gIndexList[off + k]];
        const float3 d = L.Pos - i.WorldPos;
        const float  dist = length(d);
        if (dist >= L.Radius) continue;
        const float  ndl = saturate(dot(N, d / max(dist, 1e-4)));
        float atten = saturate(1.0 - dist / L.Radius);
        atten *= atten;
        color += albedo * L.Color * (L.Intensity * ndl * atten);
    }

    return float4(pow(saturate(color), 1.0 / 2.2), 1.0);
}
