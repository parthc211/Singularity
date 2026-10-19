// RtScene.hlsl — a DXR ray-tracing library (compiled as lib_6_3).
//
// RayGen shoots a primary ray per pixel; ClosestHit shades it (lambert + a
// hardware-traced shadow ray toward the sun, and — for reflective surfaces — a
// recursive reflection ray). Because reflections are traced against the real
// acceleration structure, they correctly show geometry that is OFF-SCREEN or
// occluded — the thing screen-space reflections fundamentally cannot do. Miss
// returns a sky gradient; ShadowMiss marks a shadow ray as un-occluded.
//
// The scene is a TLAS instancing the SHARED cube + sphere meshes. ClosestHit does
// not know which mesh it hit via the SBT; instead it reads a per-instance record
// (by InstanceID()) that carries the bindless heap indices of that mesh's vertex
// and index buffers, and fetches the hit triangle's vertices through
// ResourceDescriptorHeap[]. One hit group therefore shades every instance.

struct MeshVertex {           // must match SGE::MeshVertex (48 bytes)
    float3 pos;
    float3 normal;
    float2 uv;
    float4 tangent;
};

struct InstanceData {         // must match RtScene.cpp (32 bytes)
    uint   vbIndex;           // bindless slot: StructuredBuffer<MeshVertex>
    uint   ibIndex;           // bindless slot: Buffer<uint>
    float  reflectivity;
    uint   _pad0;
    float3 color;
    uint   _pad1;
};

RaytracingAccelerationStructure gScene     : register(t0);
StructuredBuffer<InstanceData>  gInstances : register(t1);   // one per TLAS instance
RWTexture2D<float4>             gOutput    : register(u0);

cbuffer Scene : register(b0)
{
    float4x4 gInvViewProj;
    float4   gCamPos;
    float4   gSunDir;      // direction TO the sun
    uint     gBounces;     // max reflection recursion
    float3   _pad;
}

struct Payload       { float3 color; uint depth; };
struct ShadowPayload { float  lit; };

float3 SkyColor(float3 d)
{
    const float t = saturate(d.y * 0.5 + 0.5);
    return lerp(float3(0.75, 0.8, 0.9), float3(0.2, 0.36, 0.72), t);
}

[shader("raygeneration")]
void RayGen()
{
    const uint2 px  = DispatchRaysIndex().xy;
    const float2 uv = (float2(px) + 0.5) / float2(DispatchRaysDimensions().xy);
    const float2 ndc = float2(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);

    float4 world = mul(gInvViewProj, float4(ndc, 1.0, 1.0));
    const float3 origin = gCamPos.xyz;
    const float3 dir    = normalize(world.xyz / world.w - origin);

    Payload p; p.color = 0.0; p.depth = 0;
    RayDesc ray; ray.Origin = origin; ray.Direction = dir; ray.TMin = 1e-3; ray.TMax = 1e4;
    TraceRay(gScene, RAY_FLAG_NONE, 0xFF, 0, 0, 0, ray, p);

    gOutput[px] = float4(pow(saturate(p.color), 1.0 / 2.2), 1.0);
}

[shader("miss")]
void Miss(inout Payload p) { p.color = SkyColor(WorldRayDirection()); }

[shader("miss")]
void ShadowMiss(inout ShadowPayload sp) { sp.lit = 1.0; }

[shader("closesthit")]
void ClosestHit(inout Payload p, BuiltInTriangleIntersectionAttributes attr)
{
    // Per-instance record: which mesh buffers to read (bindless) and its material.
    const InstanceData inst = gInstances[InstanceID()];
    StructuredBuffer<MeshVertex> vb = ResourceDescriptorHeap[inst.vbIndex];
    Buffer<uint>                 ib = ResourceDescriptorHeap[inst.ibIndex];

    const uint   prim = PrimitiveIndex();
    const uint3  tri  = uint3(ib[prim * 3 + 0], ib[prim * 3 + 1], ib[prim * 3 + 2]);
    const float3 b    = float3(1.0 - attr.barycentrics.x - attr.barycentrics.y,
                               attr.barycentrics.x, attr.barycentrics.y);

    // Interpolate the object-space normal, then map to world. The correct normal
    // transform is the inverse-transpose of object->world; WorldToObject3x4() is
    // that inverse, so mul(n, (float3x3)WorldToObject3x4()) applies its transpose.
    const float3 nObj = vb[tri.x].normal * b.x + vb[tri.y].normal * b.y + vb[tri.z].normal * b.z;
    const float3 N    = normalize(mul(nObj, (float3x3)WorldToObject3x4()));

    const float3 albedo = inst.color;
    const float  refl   = inst.reflectivity;
    const float3 hit    = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();

    // Hardware-traced hard shadow toward the sun.
    const float3 L = normalize(gSunDir.xyz);
    ShadowPayload sp; sp.lit = 0.0;
    RayDesc sray; sray.Origin = hit + N * 1e-3; sray.Direction = L; sray.TMin = 1e-3; sray.TMax = 1e4;
    TraceRay(gScene, RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER,
             0xFF, 0, 0, /*ShadowMiss*/ 1, sray, sp);

    const float  ndl    = saturate(dot(N, L));
    const float3 direct = albedo * (0.12 + 0.88 * ndl * sp.lit);

    float3 color = direct;
    if (refl > 0.01 && p.depth < gBounces) {
        const float3 R = reflect(WorldRayDirection(), N);
        Payload rp; rp.color = 0.0; rp.depth = p.depth + 1;
        RayDesc rray; rray.Origin = hit + N * 1e-3; rray.Direction = R; rray.TMin = 1e-3; rray.TMax = 1e4;
        TraceRay(gScene, RAY_FLAG_NONE, 0xFF, 0, 0, 0, rray, rp);
        color = lerp(direct, rp.color, refl);
    }
    p.color = color;
}
