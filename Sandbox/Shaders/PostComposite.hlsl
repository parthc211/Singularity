// PostComposite.hlsl — the end of the chain: combine scene + bloom, apply
// exposure (manual or auto), tonemap, colour-grade, and the lens effects, then
// gamma-encode to the UNORM back buffer. Each effect is gated by a parameter so
// the ImGui panel can toggle the whole stack a node at a time.

cbuffer Params : register(b0)
{
    float gExposure;      // manual EV multiplier
    int   gAutoExposure;  // use the adapted-luminance target instead
    float gBloomStrength;
    float gVignette;

    float gCaStrength;    // chromatic aberration
    float gGrain;
    float gContrast;
    float gSaturation;

    float gTemperature;   // warm (+) / cool (-)
    float gKeyValue;      // middle-grey target for auto-exposure
    float gTime;          // for grain animation
    int   gGamma;
}

Texture2D<float4>  gScene   : register(t0);
Texture2D<float4>  gBloom   : register(t1);
Texture2D<float>   gAdapted : register(t2);
SamplerState       gSamp    : register(s0);

struct VSOut { float4 Pos : SV_POSITION; float2 Uv : TEXCOORD0; };

VSOut VSMain(uint vid : SV_VertexID)
{
    float2 uv = float2((vid << 1) & 2, vid & 2);
    VSOut o; o.Uv = uv; o.Pos = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    o.Pos.y = -o.Pos.y;
    return o;
}

float3 ACESFilm(float3 x)
{
    return saturate((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14));
}

float hash(float2 p) { return frac(sin(dot(p, float2(127.1, 311.7))) * 43758.5453); }

float4 PSMain(VSOut i) : SV_TARGET
{
    const float2 uv = i.Uv;

    // Chromatic aberration: fetch the scene's R/G/B at increasing radial offsets.
    const float2 dir = (uv - 0.5);
    float3 scene;
    scene.r = gScene.SampleLevel(gSamp, uv + dir * gCaStrength,        0).r;
    scene.g = gScene.SampleLevel(gSamp, uv,                            0).g;
    scene.b = gScene.SampleLevel(gSamp, uv - dir * gCaStrength,        0).b;

    float3 bloom = gBloom.SampleLevel(gSamp, uv, 0).rgb;
    float3 hdr   = scene + bloom * gBloomStrength;

    // Exposure: manual, or key-value / adapted scene luminance (eye adaptation).
    float exposure = gExposure;
    if (gAutoExposure != 0) {
        const float avg = max(gAdapted[uint2(0, 0)], 1e-4);
        exposure *= gKeyValue / avg;
    }
    hdr *= exposure;

    float3 color = ACESFilm(hdr);

    // ---- Colour grade (operates on the tonemapped [0,1] image) ----
    // Temperature: push toward warm/cool.
    color += float3(gTemperature, 0.0, -gTemperature) * 0.5;
    // Saturation around luma.
    const float luma = dot(color, float3(0.2126, 0.7152, 0.0722));
    color = lerp((float3)luma, color, gSaturation);
    // Contrast around middle grey.
    color = saturate((color - 0.5) * gContrast + 0.5);

    // ---- Lens / sensor ----
    // Vignette: darken toward the corners.
    const float d = length(uv - 0.5);
    color *= lerp(1.0, smoothstep(0.8, 0.3, d), gVignette);
    // Film grain.
    const float g = (hash(uv * 1024.0 + gTime) - 0.5) * gGrain;
    color += g;

    color = saturate(color);
    if (gGamma != 0) color = pow(color, 1.0 / 2.2);
    return float4(color, 1.0);
}
