// IblSky.hlsl — bake a procedural analytic sky into one mip of an environment
// cubemap (compute). The sky is HDR (values well above 1 around the sun), which
// is what makes image-based lighting meaningful: the irradiance/prefilter bakes
// integrate this radiance. One thread per output texel; z = cube face.
//
// Root constants (b0): face size in texels, sun direction, sun intensity.

cbuffer Params : register(b0)
{
    uint   gFaceSize;
    float3 gSunDir;        // direction TO the sun (normalized)
    float  gSunIntensity;
    float  gExposure;      // overall sky brightness scale
}

RWTexture2DArray<float4> gOut : register(u0);

// Standard D3D cube-face convention so a TextureCube SRV samples these back
// consistently. face: 0=+X 1=-X 2=+Y 3=-Y 4=+Z 5=-Z.
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

// A cheap but HDR sky: a zenith->horizon gradient, a warm horizon band, a ground
// hemisphere, plus a bright sun disk with a soft glow. Not physically derived
// (that's the future atmosphere step) — just a plausible environment to light with.
float3 SkyRadiance(float3 dir)
{
    const float t = saturate(dir.y * 0.5 + 0.5);        // 0 down, 1 up
    const float3 zenith  = float3(0.18, 0.34, 0.72);
    const float3 horizon = float3(0.72, 0.80, 0.92);
    const float3 ground  = float3(0.22, 0.20, 0.17);

    float3 sky = lerp(horizon, zenith, pow(saturate(dir.y), 0.45));
    sky = lerp(sky, ground, saturate(-dir.y * 3.0));    // below horizon -> ground

    // Sun: tight disk + broad glow, scaled way past 1.0 (HDR).
    const float cosSun = saturate(dot(dir, normalize(gSunDir)));
    const float disk    = smoothstep(0.9995, 0.9999, cosSun);
    const float glow    = pow(cosSun, 350.0) * 0.6 + pow(cosSun, 12.0) * 0.15;
    const float3 sunCol = float3(1.0, 0.95, 0.85);

    float3 col = sky * gExposure + sunCol * (disk * gSunIntensity + glow * gSunIntensity);
    return max(col, 0.0);
}

[numthreads(8, 8, 1)]
void CSMain(uint3 id : SV_DispatchThreadID)
{
    if (id.x >= gFaceSize || id.y >= gFaceSize) return;
    const float2 uv  = (float2(id.xy) + 0.5) / float(gFaceSize);
    const float3 dir = DirFromFaceUV(id.z, uv);
    gOut[uint3(id.xy, id.z)] = float4(SkyRadiance(dir), 1.0);
}
