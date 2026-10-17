// ClusterAABB.hlsl — build the view-space AABB of every froxel (frustum voxel).
//
// The view frustum is diced into a grid: gGrid.x * gGrid.y screen tiles, each
// split into gGrid.z depth slices spaced EXPONENTIALLY (so near slices are thin
// and far ones thick, matching how perspective compresses depth). One thread per
// froxel computes the eight corner rays through its screen tile, intersects them
// with its near/far depth planes, and stores the min/max. This depends only on
// projection + resolution, so it's rebuilt only when those change — not per frame.

cbuffer Params : register(b0)
{
    float4x4 gInvProj;
    uint3    gGrid;
    float    gNear;
    float    gFar;
    float2   gScreen;
}

struct AABB { float4 Min; float4 Max; };
RWStructuredBuffer<AABB> gAABB : register(u0);

// NDC (z=1) -> a view-space point; eye is at the origin so this doubles as a ray.
float3 ScreenRay(float2 ndc)
{
    float4 v = mul(gInvProj, float4(ndc, 1.0, 1.0));
    return v.xyz / v.w;
}

// Intersect the ray from the origin through `ray` with the plane z = planeZ.
float3 IntersectZ(float3 ray, float planeZ)
{
    return ray * (planeZ / ray.z);
}

[numthreads(64, 1, 1)]
void CSMain(uint3 id : SV_DispatchThreadID)
{
    const uint count = gGrid.x * gGrid.y * gGrid.z;
    const uint idx = id.x;
    if (idx >= count) return;

    const uint tileX = idx % gGrid.x;
    const uint tileY = (idx / gGrid.x) % gGrid.y;
    const uint slice = idx / (gGrid.x * gGrid.y);

    // Tile pixel rectangle -> NDC (y flipped).
    const float2 tileSize = gScreen / float2(gGrid.x, gGrid.y);
    const float2 pMin = float2(tileX, tileY) * tileSize;
    const float2 pMax = float2(tileX + 1, tileY + 1) * tileSize;
    float2 ndcMin = float2(pMin.x / gScreen.x * 2.0 - 1.0, 1.0 - pMax.y / gScreen.y * 2.0);
    float2 ndcMax = float2(pMax.x / gScreen.x * 2.0 - 1.0, 1.0 - pMin.y / gScreen.y * 2.0);

    // Exponential depth slice bounds (view space, +z forward).
    const float zNear = gNear * pow(gFar / gNear, float(slice)     / float(gGrid.z));
    const float zFar  = gNear * pow(gFar / gNear, float(slice + 1) / float(gGrid.z));

    const float3 rays[4] = {
        ScreenRay(float2(ndcMin.x, ndcMin.y)), ScreenRay(float2(ndcMax.x, ndcMin.y)),
        ScreenRay(float2(ndcMin.x, ndcMax.y)), ScreenRay(float2(ndcMax.x, ndcMax.y)),
    };

    float3 mn = 1e30, mx = -1e30;
    [unroll] for (int i = 0; i < 4; ++i) {
        const float3 a = IntersectZ(rays[i], zNear);
        const float3 b = IntersectZ(rays[i], zFar);
        mn = min(mn, min(a, b)); mx = max(mx, max(a, b));
    }

    AABB box; box.Min = float4(mn, 0); box.Max = float4(mx, 0);
    gAABB[idx] = box;
}
