// OitTransparent.hlsl — transparent geometry, two ways:
//
//   PSAccum  — weighted-blended OIT (McGuire/Bavoil). Writes two targets: an
//              ACCUM target that additively sums premultiplied colour and alpha,
//              each scaled by a depth-based WEIGHT (nearer surfaces weigh more),
//              and a REVEALAGE target that multiplicatively accumulates (1-alpha).
//              Because both use order-independent blend ops, the surfaces can be
//              drawn in ANY order and composite to the same result — the whole
//              point of OIT.
//   PSNaive  — straight alpha blending to the back buffer, order-DEPENDENT, for
//              the A/B comparison (watch the overlap colours flip as you orbit).

cbuffer Object : register(b0)
{
    float4x4 gMVP;
    float4x4 gModel;
    float4x4 gModelView;
    float4   gColor;        // rgb, a = transparency
}

struct VSOut {
    float4 Pos   : SV_POSITION;
    float3 N     : NORMAL;
    float  ViewZ : TEXCOORD0;   // positive view-space depth, for the weight
};

VSOut VSMain(float3 pos : POSITION, float3 nrm : NORMAL)
{
    VSOut o;
    o.Pos   = mul(gMVP, float4(pos, 1.0));
    o.N     = mul((float3x3)gModel, nrm);
    o.ViewZ = mul(gModelView, float4(pos, 1.0)).z;
    return o;
}

float3 Lit(float3 n)
{
    const float3 L = normalize(float3(0.4, 0.8, -0.4));
    const float  ndl = saturate(dot(normalize(n), L));
    return gColor.rgb * (0.25 + 0.75 * ndl);
}

struct AccumOut {
    float4 Accum    : SV_TARGET0;   // RGBA16F, additive
    float  Revealage: SV_TARGET1;   // R16F, multiplicative (1-alpha)
};

AccumOut PSAccum(VSOut i)
{
    const float3 c = Lit(i.N);
    const float  a = gColor.a;
    // Depth weight: nearer fragments dominate. Tuned for this scene's depth range.
    const float  w = clamp(0.03 / (1e-5 + pow(i.ViewZ / 20.0, 4.0)), 1e-2, 3e3) * a;

    AccumOut o;
    o.Accum     = float4(c * a, a) * w;   // premultiplied colour, weighted
    o.Revealage = a;
    return o;
}

float4 PSNaive(VSOut i) : SV_TARGET
{
    return float4(Lit(i.N), gColor.a);    // order-dependent alpha blend
}
