// GpuDrivenCull.hlsl — GPU frustum culling that builds the indirect draw list.
//
// One thread per instance tests the instance's bounding sphere against the six
// frustum planes; survivors atomically append a DrawIndexedInstanced command
// (plus the instance index as a root constant) into the commands buffer and bump
// a counter. ExecuteIndirect then issues exactly the surviving draws from that
// buffer — the CPU never touches per-object visibility or draw calls. CSReset
// zeroes the counter each frame.

struct Instance {
    float4x4 model;
    float4   color;
    float4   sphere;   // xyz centre, w radius (world space)
};

// Matches the C++ IndirectCommand: a root constant + D3D12_DRAW_INDEXED_ARGUMENTS.
struct IndirectCommand {
    uint instanceIndex;
    uint indexCount;
    uint instanceCount;
    uint startIndex;
    int  baseVertex;
    uint startInstance;
};

cbuffer Cull : register(b0)
{
    float4 gFrustum[6];     // normalized planes (xyz = normal, w = distance)
    uint   gInstanceCount;
    uint   gIndexCount;
    uint   gCullEnabled;    // 0 => pass everything (A/B comparison)
    uint   _pad;
}

StructuredBuffer<Instance>           gInstances : register(t0);
RWStructuredBuffer<IndirectCommand>  gCommands  : register(u0);
RWStructuredBuffer<uint>             gCounter   : register(u1);

[numthreads(1, 1, 1)]
void CSReset(uint3 id : SV_DispatchThreadID) { gCounter[0] = 0; }

bool Visible(float4 sphere)
{
    if (gCullEnabled == 0) return true;
    [unroll] for (int i = 0; i < 6; ++i)
        if (dot(gFrustum[i].xyz, sphere.xyz) + gFrustum[i].w < -sphere.w)
            return false;   // fully outside this plane
    return true;
}

[numthreads(64, 1, 1)]
void CSMain(uint3 id : SV_DispatchThreadID)
{
    const uint i = id.x;
    if (i >= gInstanceCount) return;
    if (!Visible(gInstances[i].sphere)) return;

    uint slot;
    InterlockedAdd(gCounter[0], 1, slot);

    IndirectCommand c;
    c.instanceIndex = i;
    c.indexCount    = gIndexCount;
    c.instanceCount = 1;
    c.startIndex    = 0;
    c.baseVertex    = 0;
    c.startInstance = 0;
    gCommands[slot] = c;
}
