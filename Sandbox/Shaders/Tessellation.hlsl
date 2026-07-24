// Hardware tessellation: a coarse grid of quad patches is subdivided on the GPU,
// finer near the camera (LOD), and each generated vertex is displaced by a
// procedural fBm height to form terrain. Pipeline: VS (pass control points) ->
// Hull (tess factors + control points) -> fixed-function tessellator -> Domain
// (interpolate + displace + project) -> PS (shade).

cbuffer TessParams : register(b0)
{
    float4x4 gViewProj;    // world -> clip (terrain control points are already world-space)
    float3   gCameraPos;   float gMaxTess;     // LOD clamp
    float    gLodScale;    // larger => more subdivision at a given distance
    float    gHeightScale; // displacement amplitude
    float    gNoiseFreq;   // base terrain frequency
    float    _pad;
};

// ----------------------------- Vertex shader -----------------------------
struct VSInput  { float3 Pos : POSITION; };  // one patch control point (corner), Y=0
struct VSOutput { float3 WorldPos : POSITION; };

VSOutput VSMain(VSInput i) {
    VSOutput o;
    o.WorldPos = i.Pos; // pass through; displacement happens in the domain shader
    return o;
}

// ----------------------------- Hull shader -------------------------------
// Tess factor from the distance of an edge midpoint to the camera. Because
// adjacent patches share an edge (and thus its midpoint), they compute the SAME
// factor for it — so the seams match and there are no cracks.
float EdgeTess(float3 a, float3 b) {
    float3 mid = 0.5 * (a + b);
    float  d   = distance(gCameraPos, mid);
    return clamp(gLodScale / max(d, 1.0), 1.0, gMaxTess);
}

struct PatchConstants {
    float Edges[4]  : SV_TessFactor;
    float Inside[2] : SV_InsideTessFactor;
};

// Control point order (matches the index buffer): 0=(u0,v0) 1=(u1,v0) 2=(u1,v1) 3=(u0,v1).
PatchConstants ConstantHS(InputPatch<VSOutput, 4> ip) {
    PatchConstants c;
    c.Edges[0] = EdgeTess(ip[0].WorldPos, ip[3].WorldPos); // u = 0 edge
    c.Edges[1] = EdgeTess(ip[0].WorldPos, ip[1].WorldPos); // v = 0 edge
    c.Edges[2] = EdgeTess(ip[1].WorldPos, ip[2].WorldPos); // u = 1 edge
    c.Edges[3] = EdgeTess(ip[3].WorldPos, ip[2].WorldPos); // v = 1 edge
    c.Inside[0] = 0.5 * (c.Edges[1] + c.Edges[3]);
    c.Inside[1] = 0.5 * (c.Edges[0] + c.Edges[2]);
    return c;
}

[domain("quad")]
[partitioning("fractional_odd")]     // smooth, pop-free LOD changes
[outputtopology("triangle_cw")]
[outputcontrolpoints(4)]
[patchconstantfunc("ConstantHS")]
VSOutput HSMain(InputPatch<VSOutput, 4> ip, uint id : SV_OutputControlPointID) {
    return ip[id]; // pass control points unchanged
}

// ----------------------------- Domain shader -----------------------------
// Runs once per tessellated vertex. SV_DomainLocation is the (u,v) within the
// patch; we bilerp the corners to a world position, displace Y by noise, project.
float Hash(float2 p)       { return frac(sin(dot(p, float2(127.1, 311.7))) * 43758.5453); }
float ValueNoise(float2 p) {
    float2 i = floor(p), f = frac(p);
    float2 u = f * f * (3.0 - 2.0 * f);
    float a = Hash(i), b = Hash(i + float2(1, 0)), c = Hash(i + float2(0, 1)), d = Hash(i + float2(1, 1));
    return lerp(lerp(a, b, u.x), lerp(c, d, u.x), u.y);
}
float TerrainHeight(float2 xz) {
    float h = 0.0, amp = 0.5, freq = gNoiseFreq;
    [unroll] for (int o = 0; o < 5; ++o) { h += amp * ValueNoise(xz * freq); freq *= 2.0; amp *= 0.5; }
    return h;
}

struct DSOutput {
    float4 Pos      : SV_POSITION;
    float3 WorldPos : POSITION;
    float  Height   : TEXCOORD0;
};

[domain("quad")]
DSOutput DSMain(PatchConstants c, float2 uv : SV_DomainLocation, OutputPatch<VSOutput, 4> patch) {
    float3 bottom = lerp(patch[0].WorldPos, patch[1].WorldPos, uv.x);
    float3 top    = lerp(patch[3].WorldPos, patch[2].WorldPos, uv.x);
    float3 world  = lerp(bottom, top, uv.y);
    world.y = TerrainHeight(world.xz) * gHeightScale;

    DSOutput o;
    o.WorldPos = world;
    o.Height   = world.y;
    o.Pos      = mul(gViewProj, float4(world, 1.0)); // mul(M,v) per engine convention
    return o;
}

// ----------------------------- Pixel shader ------------------------------
float4 PSMain(DSOutput i) : SV_TARGET {
    // Normal from the height field via central differences.
    const float e = 0.5;
    float hl = TerrainHeight(i.WorldPos.xz + float2(-e, 0)) * gHeightScale;
    float hr = TerrainHeight(i.WorldPos.xz + float2( e, 0)) * gHeightScale;
    float hd = TerrainHeight(i.WorldPos.xz + float2(0, -e)) * gHeightScale;
    float hu = TerrainHeight(i.WorldPos.xz + float2(0,  e)) * gHeightScale;
    float3 n = normalize(float3(hl - hr, 2.0 * e, hd - hu));

    float ndl = saturate(dot(n, normalize(float3(0.4, 1.0, 0.3))));

    // Altitude colouring: grass -> rock -> snow.
    float t = saturate(i.Height / max(gHeightScale, 1e-3));
    float3 grass = float3(0.20, 0.40, 0.15);
    float3 rock  = float3(0.45, 0.35, 0.25);
    float3 snow  = float3(0.92, 0.93, 0.97);
    float3 col   = (t < 0.5) ? lerp(grass, rock, t * 2.0) : lerp(rock, snow, (t - 0.5) * 2.0);

    return float4(col * (0.25 + 0.75 * ndl), 1.0);
}
