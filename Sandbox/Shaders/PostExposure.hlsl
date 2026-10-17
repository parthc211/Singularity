// PostExposure.hlsl — auto-exposure / eye adaptation (compute).
//
// One thread samples a coarse grid of the HDR scene, forms the geometric-mean
// luminance (average of log-luminance, exp'd back — the standard perceptual
// average), then blends the stored adapted value TOWARD it at a rate scaled by
// dt, so the "eye" adjusts smoothly when the scene brightness changes. The 1x1
// result persists across frames (read-modify-write of the single UAV texel) and
// the composite divides by it to pick an exposure.

cbuffer Params : register(b0)
{
    uint2 gDim;         // HDR target size
    float gAdaptSpeed;  // larger = faster adaptation
    float gDt;          // seconds since last frame
}

Texture2D<float4>  gScene   : register(t0);
RWTexture2D<float> gAdapted : register(u0);
SamplerState       gSamp    : register(s0);

[numthreads(1, 1, 1)]
void CSMain(uint3 id : SV_DispatchThreadID)
{
    const int kGrid = 16;
    float logSum = 0.0;
    for (int y = 0; y < kGrid; ++y)
        for (int x = 0; x < kGrid; ++x) {
            const float2 uv = (float2(x, y) + 0.5) / float(kGrid);
            const float3 c  = gScene.SampleLevel(gSamp, uv, 0).rgb;
            const float lum = max(dot(c, float3(0.2126, 0.7152, 0.0722)), 1e-4);
            logSum += log(lum);
        }
    // Clamp the metered luminance so a mostly-black scene (whose log-average
    // collapses toward zero) can't drive the exposure to a blown-out extreme.
    const float cur = clamp(exp(logSum / float(kGrid * kGrid)), 0.1, 60.0);

    float prev = gAdapted[uint2(0, 0)];
    // Reject an uninitialized/garbage first-frame value (the target isn't cleared).
    if (!(prev > 1e-4 && prev < 1e4)) prev = cur;
    const float t = saturate(1.0 - exp(-gDt * gAdaptSpeed));
    gAdapted[uint2(0, 0)] = lerp(prev, cur, t);
}
