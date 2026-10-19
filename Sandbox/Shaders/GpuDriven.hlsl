// GpuDriven.hlsl — the indirect draw. Each ExecuteIndirect command sets the
// per-draw instance index as a root constant (b0); the vertex shader reads that
// instance's transform/colour from a structured buffer. One mesh, thousands of
// instances, all issued by a single ExecuteIndirect of the GPU-culled list.

cbuffer DrawRoot : register(b0) { uint gInstanceIndex; uint3 _pad; }   // set per indirect draw
cbuffer Frame    : register(b1) { float4x4 gViewProj; float4 gLightDir; } // .w > 0.5 => face-colour debug

struct Instance { float4x4 model; float4 color; float4 sphere; };
StructuredBuffer<Instance> gInstances : register(t0);

struct VSOut {
    float4 Pos   : SV_POSITION;
    float3 N     : NORMAL;
    float3 Color : COLOR0;   // per-instance colour
    float3 Face  : COLOR1;   // per-face colour (debug)
};

// Per-face debug colour derived from the (axis-aligned) object-space normal, so
// the shared cube.obj needs no custom per-vertex attribute:
// +X red, -X green, +Y(top) blue, -Y(bottom) YELLOW, +Z magenta, -Z cyan.
float3 FaceColor(float3 n)
{
    const float3 a = abs(n);
    if (a.x >= a.y && a.x >= a.z) return n.x > 0.0 ? float3(1,0,0) : float3(0,1,0);
    if (a.y >= a.z)               return n.y > 0.0 ? float3(0,0,1) : float3(1,1,0);
    return n.z > 0.0 ? float3(1,0,1) : float3(0,1,1);
}

// Inputs are a prefix of MeshVertex (POSITION, NORMAL); the 48-byte stride from
// the vertex-buffer view skips the unused TEXCOORD/TANGENT fields.
VSOut VSMain(float3 pos : POSITION, float3 nrm : NORMAL)
{
    const Instance inst = gInstances[gInstanceIndex];
    const float3 world = mul(inst.model, float4(pos, 1.0)).xyz;
    VSOut o;
    o.Pos   = mul(gViewProj, float4(world, 1.0));
    o.N     = mul((float3x3)inst.model, nrm);
    o.Color = inst.color.rgb;
    o.Face  = FaceColor(nrm);   // object-space face identity
    return o;
}

float4 PSMain(VSOut i) : SV_TARGET
{
    // Debug: flat per-face colours so each visible face is unmistakable.
    // +X red, -X green, +Y(top) blue, -Y(bottom) YELLOW, +Z magenta, -Z cyan.
    if (gLightDir.w > 0.5) return float4(i.Face, 1.0);

    const float3 L = normalize(gLightDir.xyz);
    const float  ndl = saturate(dot(normalize(i.N), L));
    return float4(i.Color * (0.2 + 0.8 * ndl), 1.0);
}
