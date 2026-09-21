// IblIrradiance.hlsl — diffuse IBL bake (compute).
//
// For each output direction N, integrate incoming radiance from the environment
// over the cosine-weighted hemisphere around N. The result is the irradiance a
// Lambertian surface facing N receives; at shade time diffuse = irradiance*albedo.
// Output is a small cubemap (detail isn't needed — irradiance is very smooth).

cbuffer Params : register(b0)
{
    uint gFaceSize;
    uint _pad;
}

TextureCube<float4>      gEnv : register(t0);
RWTexture2DArray<float4> gOut : register(u0);
SamplerState             gSamp : register(s0);

static const float PI = 3.14159265359;

float3 DirFromFaceUV(uint face, float2 uv)
{
    float2 st = uv * 2.0 - 1.0;
    float3 d;
    if      (face == 0) d = float3( 1.0, -st.y, -st.x);
    else if (face == 1) d = float3(-1.0, -st.y,  st.x);
    else if (face == 2) d = float3( st.x,  1.0,  st.y);
    else if (face == 3) d = float3( st.x, -1.0, -st.y);
    else if (face == 4) d = float3( st.x, -st.y,  1.0);
    else                d = float3(-st.x, -st.y, -1.0);
    return normalize(d);
}

[numthreads(8, 8, 1)]
void CSMain(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= gFaceSize || id.y >= gFaceSize) return;
    const float2 uv = (float2(id.xy) + 0.5) / float(gFaceSize);
    const float3 N  = DirFromFaceUV(id.z, uv);

    // Tangent basis around N.
    float3 up = abs(N.y) < 0.999 ? float3(0, 1, 0) : float3(1, 0, 0);
    const float3 right = normalize(cross(up, N));
    up = cross(N, right);

    // Riemann sum over the hemisphere (phi around, theta up). Env is sampled at a
    // low mip (radiance is being averaged anyway) to cut variance and cost.
    float3 irradiance = 0.0;
    float  samples    = 0.0;
    const float dPhi   = 0.025;
    const float dTheta = 0.025;
    for (float phi = 0.0; phi < 2.0 * PI; phi += dPhi) {
        for (float theta = 0.0; theta < 0.5 * PI; theta += dTheta) {
            // Spherical (tangent space) -> world.
            const float3 t = float3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            const float3 dir = t.x * right + t.y * up + t.z * N;
            irradiance += gEnv.SampleLevel(gSamp, dir, 2).rgb * cos(theta) * sin(theta);
            samples += 1.0;
        }
    }
    irradiance = PI * irradiance / max(samples, 1.0);
    gOut[uint3(id.xy, id.z)] = float4(irradiance, 1.0);
}
