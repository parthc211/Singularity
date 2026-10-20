// MeshShader.hlsl — the amplification -> mesh -> pixel geometry front end.
//
// The CPU issues one DispatchMesh of ceil(meshletCount / AS_GROUP) amplification
// groups and nothing else. Each AS thread tests one meshlet (frustum + normal-cone
// back-face cull) and the group dispatches only the survivors as mesh-shader
// groups. Each mesh-shader group expands one meshlet into its vertices + triangles
// directly, with no input assembler. This is per-cluster culling on the GPU — the
// Nanite-adjacent path — instead of per-object culling on the CPU.

#define AS_GROUP 32     // meshlets tested per amplification group
#define MS_GROUP 128    // threads per mesh group (covers <=64 verts and <=124 prims)
#define MAX_VERTS 64
#define MAX_PRIMS 124

struct MeshVertex {           // matches SGE::MeshVertex (48 bytes)
    float3 pos;
    float3 normal;
    float2 uv;
    float4 tangent;
};
struct Meshlet       { uint vertOffset; uint vertCount; uint primOffset; uint primCount; };
struct MeshletBounds { float3 center; float radius; float3 coneAxis; float coneCutoff; };

cbuffer Frame : register(b0)
{
    float4x4 gViewProj;
    float4x4 gModel;
    float4   gFrustum[6];   // world-space planes (xyz = normal, w = d)
    float4   gCamPos;       // xyz
    uint     gMeshletCount;
    uint     gCullEnabled;
    uint     gColorMode;    // 1 => tint each meshlet a distinct colour
    uint     _pad;
};

StructuredBuffer<MeshVertex>    gVertices      : register(t0);
StructuredBuffer<Meshlet>       gMeshlets      : register(t1);
StructuredBuffer<MeshletBounds> gBounds        : register(t2);
StructuredBuffer<uint>          gVertexIndices : register(t3);
StructuredBuffer<uint>          gPrimIndices   : register(t4);

struct Payload { uint MeshletIndices[AS_GROUP]; };

struct VSOut {
    float4 Pos   : SV_POSITION;
    float3 N     : NORMAL;
    float3 Color : COLOR0;   // per-meshlet debug colour
};

float3 MeshletColor(uint i)
{
    uint h = i * 1664525u + 1013904223u;
    return float3((h & 255) / 255.0, ((h >> 8) & 255) / 255.0, ((h >> 16) & 255) / 255.0);
}

// Frustum + normal-cone test for one meshlet, in world space.
bool MeshletVisible(uint mi)
{
    MeshletBounds b = gBounds[mi];
    const float3 center = mul(gModel, float4(b.center, 1.0)).xyz;
    const float  radius = b.radius;   // model is rotation + translation (no scale)

    [unroll] for (int i = 0; i < 6; ++i)
        if (dot(gFrustum[i].xyz, center) + gFrustum[i].w < -radius)
            return false;             // fully outside this plane

    // Normal-cone back-face rejection (coneCutoff > 1 disables it).
    const float3 axis = normalize(mul((float3x3)gModel, b.coneAxis));
    const float3 view = normalize(center - gCamPos.xyz);
    if (dot(axis, view) > b.coneCutoff)
        return false;

    return true;
}

// ---- Amplification shader ----
groupshared Payload s_payload;
groupshared uint    s_count;

[numthreads(AS_GROUP, 1, 1)]
void ASMain(uint dtid : SV_DispatchThreadID, uint gtid : SV_GroupThreadID)
{
    if (gtid == 0) s_count = 0;
    GroupMemoryBarrierWithGroupSync();

    bool visible = false;
    if (dtid < gMeshletCount)
        visible = (gCullEnabled == 0) || MeshletVisible(dtid);

    if (visible) {
        uint slot;
        InterlockedAdd(s_count, 1, slot);
        s_payload.MeshletIndices[slot] = dtid;
    }
    GroupMemoryBarrierWithGroupSync();

    DispatchMesh(s_count, 1, 1, s_payload);
}

// ---- Mesh shader ----
[outputtopology("triangle")]
[numthreads(MS_GROUP, 1, 1)]
void MSMain(
    uint gtid : SV_GroupThreadID,
    uint3 gid : SV_GroupID,
    in payload Payload pl,
    out vertices VSOut verts[MAX_VERTS],
    out indices  uint3 tris[MAX_PRIMS])
{
    const uint    mi = pl.MeshletIndices[gid.x];
    const Meshlet m  = gMeshlets[mi];
    SetMeshOutputCounts(m.vertCount, m.primCount);

    if (gtid < m.vertCount) {
        const uint       gv = gVertexIndices[m.vertOffset + gtid];
        const MeshVertex v  = gVertices[gv];
        VSOut o;
        o.Pos   = mul(gViewProj, mul(gModel, float4(v.pos, 1.0)));
        o.N     = mul((float3x3)gModel, v.normal);
        o.Color = MeshletColor(mi);
        verts[gtid] = o;
    }
    if (gtid < m.primCount) {
        const uint packed = gPrimIndices[m.primOffset + gtid];
        tris[gtid] = uint3(packed & 0x3FF, (packed >> 10) & 0x3FF, (packed >> 20) & 0x3FF);
    }
}

// ---- Pixel shader ----
float4 PSMain(VSOut i) : SV_TARGET
{
    const float3 L   = normalize(float3(0.5, 0.8, 0.3));
    const float  ndl = saturate(dot(normalize(i.N), L));
    const float3 base = (gColorMode != 0) ? i.Color : float3(0.80, 0.80, 0.82);
    return float4(base * (0.25 + 0.75 * ndl), 1.0);
}
