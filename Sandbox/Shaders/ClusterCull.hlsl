// ClusterCull.hlsl — assign lights to froxels (compute).
//
// One thread per froxel tests every light's bounding sphere against the froxel's
// view-space AABB and, for those that overlap, appends their indices to a single
// global index list (carved out atomically) while recording this froxel's
// (offset, count) in the light grid. The forward shade pass then loops only the
// handful of lights its froxel owns instead of all of them — the whole point of
// clustered shading. CSReset zeroes the global allocation counter each frame.

struct Light { float3 Pos; float Radius; float3 Color; float Intensity; };
struct AABB  { float4 Min; float4 Max; };

cbuffer Params : register(b0)
{
    float4x4 gView;
    uint3    gGrid;
    uint     gLightCount;
    uint     gMaxIndices;
}

StructuredBuffer<Light>   gLights    : register(t0);
StructuredBuffer<AABB>    gAABB      : register(t1);
RWStructuredBuffer<uint2> gLightGrid : register(u0);   // (offset, count) per froxel
RWStructuredBuffer<uint>  gIndexList : register(u1);
RWStructuredBuffer<uint>  gCounter   : register(u2);

[numthreads(1, 1, 1)]
void CSReset(uint3 id : SV_DispatchThreadID) { gCounter[0] = 0; }

// Squared distance from a point to an AABB; 0 inside. Sphere overlaps when <= r^2.
bool SphereVsAABB(float3 c, float r, AABB box)
{
    float3 d = max(max(box.Min.xyz - c, c - box.Max.xyz), 0.0);
    return dot(d, d) <= r * r;
}

[numthreads(64, 1, 1)]
void CSMain(uint3 id : SV_DispatchThreadID)
{
    const uint froxels = gGrid.x * gGrid.y * gGrid.z;
    const uint idx = id.x;
    if (idx >= froxels) return;

    const AABB box = gAABB[idx];

    // Pass 1: count overlapping lights.
    uint count = 0;
    for (uint i = 0; i < gLightCount; ++i) {
        const float3 vp = mul(gView, float4(gLights[i].Pos, 1.0)).xyz;
        if (SphereVsAABB(vp, gLights[i].Radius, box)) ++count;
    }

    // Reserve a contiguous run in the shared index list.
    uint offset = 0;
    if (count > 0) InterlockedAdd(gCounter[0], count, offset);
    if (offset + count > gMaxIndices) { gLightGrid[idx] = uint2(offset, 0); return; }
    gLightGrid[idx] = uint2(offset, count);

    // Pass 2: write the indices.
    uint w = offset;
    for (uint j = 0; j < gLightCount && w < offset + count; ++j) {
        const float3 vp = mul(gView, float4(gLights[j].Pos, 1.0)).xyz;
        if (SphereVsAABB(vp, gLights[j].Radius, box)) gIndexList[w++] = j;
    }
}
