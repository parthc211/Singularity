#include "Scenes/NormalMapScene.h"

#include "Core/Camera.h"
#include "Core/Logger.h"
#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/DX12/RootSignatureBuilder.h"

#include "imgui.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>

using namespace SGE;
using namespace DirectX;

namespace {

// Must match the cbuffers in NormalMap.hlsl.
struct ObjCB {
    XMFLOAT4X4 MVP;
    XMFLOAT4X4 Model;
    XMFLOAT4   UvTiling;
    XMFLOAT4   Material;   // x = roughness mult, y = metallic override (<0 = map)
};
struct FrameCB {
    XMFLOAT4 LightDir;
    XMFLOAT4 CameraPos;
    float    NormalStrength;
    int      UseNormalMap;
    int      FlipGreen;
    int      ViewMode;
    int      GammaCorrect;
    float    HeightScale;
    int      UsePom;
    int      PomShadow;
    int      PomSteps;
    float    RoughnessScale;
    int      UseMaterialMap;
    float    _pad;
};

XMVECTOR ComputeLightDir(float elevationDeg, float azimuth) {
    const float el = XMConvertToRadians(elevationDeg);
    return XMVector3Normalize(XMVectorSet(cosf(el) * cosf(azimuth), -sinf(el),
                                          cosf(el) * sinf(azimuth), 0.0f));
}

// --- procedural brick textures ------------------------------------------------
// A periodic brick height field (bevelled bricks, recessed mortar, hashed
// per-brick variation) drives both maps: the albedo colors brick vs mortar by
// the same mask, and the normal map is the height field's central-difference
// gradient. Everything wraps, so the texture tiles seamlessly.

constexpr int      kTexSize   = 512;
constexpr int      kBrickW    = 64;
constexpr int      kBrickH    = 32;
constexpr int      kMortar    = 3;   // half-width of the mortar groove, px
constexpr int      kBevel     = 4;   // ramp from mortar depth to brick top, px
constexpr uint32_t kRows      = kTexSize / kBrickH; // even -> parity wraps seamlessly

uint32_t Hash(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352dU;
    x ^= x >> 15; x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}
float Hash01(uint32_t x, uint32_t y, uint32_t salt) {
    return float(Hash(x * 73856093U ^ y * 19349663U ^ salt * 83492791U) & 0xFFFFFF)
         / float(0xFFFFFF);
}

// Height in [0,1] at wrapped texel coordinates; also reports which brick the
// texel belongs to (for per-brick color/height variation).
float BrickHeight(int px, int py, uint32_t& outBrickX, uint32_t& outBrickY) {
    px = ((px % kTexSize) + kTexSize) % kTexSize;
    py = ((py % kTexSize) + kTexSize) % kTexSize;

    const int row  = py / kBrickH;
    const int xoff = (row % 2) ? kBrickW / 2 : 0;     // running bond offset
    const int gx   = px + xoff;
    const int lx   = gx % kBrickW;
    const int ly   = py % kBrickH;
    outBrickX = uint32_t(gx / kBrickW);
    outBrickY = uint32_t(row);

    // Distance from the cell boundary; the mortar groove straddles it.
    const int dx = lx < kBrickW - lx ? lx : kBrickW - 1 - lx;
    const int dy = ly < kBrickH - ly ? ly : kBrickH - 1 - ly;
    const int d  = dx < dy ? dx : dy;

    float h;
    if (d < kMortar)                 h = 0.0f;                              // groove
    else if (d < kMortar + kBevel)   h = float(d - kMortar) / float(kBevel); // bevel ramp
    else                             h = 1.0f;

    // Per-brick height variation + fine surface noise.
    h *= 0.85f + 0.15f * Hash01(outBrickX, outBrickY, 1);
    h += 0.06f * (Hash01(uint32_t(px), uint32_t(py), 2) - 0.5f);
    return h;
}

Image MakeBrickAlbedo() {
    Image img;
    img.width = img.height = kTexSize;
    img.pixels.resize(size_t(kTexSize) * kTexSize * 4);

    for (int y = 0; y < kTexSize; ++y) {
        for (int x = 0; x < kTexSize; ++x) {
            uint32_t bx, by;
            const float h    = BrickHeight(x, y, bx, by);
            const float mask = h < 0.15f ? 0.0f : 1.0f;   // mortar vs brick

            // Hashed per-brick tint around a terracotta base; light gray mortar.
            const float tint  = 0.80f + 0.35f * Hash01(bx, by, 3);
            const float noise = 0.92f + 0.16f * Hash01(uint32_t(x), uint32_t(y), 4);
            float r = mask * (0.62f * tint) + (1 - mask) * 0.66f;
            float g = mask * (0.28f * tint) + (1 - mask) * 0.64f;
            float b = mask * (0.23f * tint) + (1 - mask) * 0.60f;
            r *= noise; g *= noise; b *= noise;

            uint8_t* d = &img.pixels[(size_t(y) * kTexSize + x) * 4];
            d[0] = uint8_t(std::min(r, 1.0f) * 255.0f + 0.5f);
            d[1] = uint8_t(std::min(g, 1.0f) * 255.0f + 0.5f);
            d[2] = uint8_t(std::min(b, 1.0f) * 255.0f + 0.5f);
            d[3] = 255;
        }
    }
    return img;
}

Image MakeBrickNormal() {
    Image img;
    img.width = img.height = kTexSize;
    img.pixels.resize(size_t(kTexSize) * kTexSize * 4);

    // Central differences over the (wrapping) height field. The image +Y axis is
    // the UV +V axis, which is what the mesh handedness reconstructs as B —
    // no green-channel flip needed for our own maps. The ALPHA channel stores
    // the height itself (1 = brick top, 0 = mortar) for parallax occlusion.
    const float scale = 2.5f;  // groove depth in texel units
    for (int y = 0; y < kTexSize; ++y) {
        for (int x = 0; x < kTexSize; ++x) {
            uint32_t bx, by;
            const float h  = BrickHeight(x, y, bx, by);
            const float hL = BrickHeight(x - 1, y, bx, by);
            const float hR = BrickHeight(x + 1, y, bx, by);
            const float hU = BrickHeight(x, y - 1, bx, by);
            const float hD = BrickHeight(x, y + 1, bx, by);

            float nx = (hL - hR) * 0.5f * scale;
            float ny = (hU - hD) * 0.5f * scale;
            float nz = 1.0f;
            const float inv = 1.0f / std::sqrt(nx * nx + ny * ny + nz * nz);
            nx *= inv; ny *= inv; nz *= inv;

            uint8_t* d = &img.pixels[(size_t(y) * kTexSize + x) * 4];
            d[0] = uint8_t((nx * 0.5f + 0.5f) * 255.0f + 0.5f);
            d[1] = uint8_t((ny * 0.5f + 0.5f) * 255.0f + 0.5f);
            d[2] = uint8_t((nz * 0.5f + 0.5f) * 255.0f + 0.5f);
            d[3] = uint8_t(std::clamp(h, 0.0f, 1.0f) * 255.0f + 0.5f);
        }
    }
    return img;
}

// ORM material map, glTF packing: R = ambient occlusion (crevices dark),
// G = roughness (mortar rough, brick faces glossier with per-brick variation),
// B = metalness (bricks are dielectric: 0). Linear data — never sRGB.
Image MakeBrickORM() {
    Image img;
    img.width = img.height = kTexSize;
    img.pixels.resize(size_t(kTexSize) * kTexSize * 4);

    for (int y = 0; y < kTexSize; ++y) {
        for (int x = 0; x < kTexSize; ++x) {
            uint32_t bx, by;
            const float h = std::clamp(BrickHeight(x, y, bx, by), 0.0f, 1.0f);

            const float ao    = 0.45f + 0.55f * h;   // grooves occluded, tops open
            const float rough = h < 0.15f
                              ? 0.92f                                        // mortar
                              : 0.42f + 0.30f * Hash01(bx, by, 5)
                                      + 0.10f * (Hash01(uint32_t(x), uint32_t(y), 6) - 0.5f);

            uint8_t* d = &img.pixels[(size_t(y) * kTexSize + x) * 4];
            d[0] = uint8_t(std::clamp(ao,    0.0f, 1.0f) * 255.0f + 0.5f);
            d[1] = uint8_t(std::clamp(rough, 0.0f, 1.0f) * 255.0f + 0.5f);
            d[2] = 0;
            d[3] = 255;
        }
    }
    return img;
}

} // namespace

const char* NormalMapScene::Description() const {
    return "The material pipeline end to end: sRGB albedo, tangent-space "
           "normal map (height in alpha), parallax occlusion mapping with "
           "contact shadows, and an ORM map (AO / roughness / metalness, glTF "
           "packing) driving a Cook-Torrance GGX BRDF — Fresnel-Schlick, "
           "Smith geometry, energy-aware diffuse. Textures load from Assets/ "
           "or fall back to the procedural brick generator. A/B every step.";
}

bool NormalMapScene::BuildTextures(const DemoContext& ctx) {
    ID3D12Device*       device = ctx.device;
    ID3D12CommandQueue* queue  = ctx.renderer->GetCommandQueue();

    // Prefer real assets when all are present; otherwise bake the brick.
    const wchar_t* albedoPath = L"Assets/BrickAlbedo.png";
    const wchar_t* normalPath = L"Assets/BrickNormal.png";
    const wchar_t* ormPath    = L"Assets/BrickORM.png";
    m_fromFiles = std::filesystem::exists(albedoPath)
               && std::filesystem::exists(normalPath)
               && std::filesystem::exists(ormPath)
               && m_albedo.CreateFromFile(device, queue, albedoPath, true)
               && m_normal.CreateFromFile(device, queue, normalPath, false)
               && m_orm.CreateFromFile(device, queue, ormPath, false);   // linear data

    if (!m_fromFiles) {
        if (!m_albedo.CreateFromImage(device, queue, MakeBrickAlbedo(), true) ||
            !m_normal.CreateFromImage(device, queue, MakeBrickNormal(), false) ||
            !m_orm.CreateFromImage(device, queue, MakeBrickORM(), false))
            return false;
        LogInfo("NormalMapScene: using procedural brick textures (drop "
                "BrickAlbedo/BrickNormal/BrickORM .png into Assets/ to override)");
    }

    // All SRVs side by side in one shader-visible heap = one descriptor table.
    if (!m_srvs.Create(device, 3))
        return false;
    m_srvs.Write(device, 0, m_albedo);
    m_srvs.Write(device, 1, m_normal);
    m_srvs.Write(device, 2, m_orm);
    return true;
}

bool NormalMapScene::BuildPipeline(const DemoContext& ctx) {
    ID3D12Device* device = ctx.device;
    m_shaders.Initialize(L"Shaders");

    // b0 object CBV (param 0), b1 frame CBV (param 1), SRV table t0..t2
    // (param 2: albedo, normal+height, ORM), static aniso wrap sampler at s0.
    if (!RootSignatureBuilder()
             .Cbv(0)
             .Cbv(1)
             .SrvTable(0, 3)
             .SamplerAnisoWrap(0)
             .Build(device, m_rootSig))
        return false;

    auto vs = m_shaders.GetOrCompile(L"NormalMap.hlsl", "VSMain", "vs_6_0");
    auto ps = m_shaders.GetOrCompile(L"NormalMap.hlsl", "PSMain", "ps_6_0");
    if (!vs || !ps) return false;

    GraphicsPipelineDesc pd;
    pd.rootSignature = m_rootSig.Get();
    pd.vs = vs; pd.ps = ps;
    pd.depthEnable = true;
    pd.rtvFormat   = DXGI_FORMAT_R8G8B8A8_UNORM;
    return m_pso.Create(device, pd);
}

void NormalMapScene::BuildObjects() {
    m_objects.clear();
    // Ground slab: tiled so the bricks stay near their authored texel density.
    m_objects.push_back({ { 14.0f, 0.5f, 14.0f }, { 0.0f, -0.25f, 0.0f }, { 7.0f, 7.0f }, false });
    // Static boxes; the tall one is forced metallic (GGX/Fresnel reference —
    // same brick maps, conductor response).
    m_objects.push_back({ { 2.0f, 2.0f, 2.0f },  { -3.5f, 1.0f,  1.5f }, { 1.0f, 1.0f }, false });
    m_objects.push_back({ { 1.4f, 2.8f, 1.4f },  {  3.0f, 1.4f, -0.5f }, { 0.7f, 1.4f }, false,
                          /*Metallic*/ 1.0f, /*RoughMult*/ 0.55f });
    // Spinning cube: shows the TBN following the surface as it rotates.
    m_objects.push_back({ { 1.6f, 1.6f, 1.6f },  {  0.0f, 2.4f,  3.0f }, { 1.0f, 1.0f }, true  });
}

void NormalMapScene::OnLoad(const DemoContext& ctx) {
    BuildObjects();
    m_ready = BuildTextures(ctx) && BuildPipeline(ctx);
}

void NormalMapScene::OnUnload() {
    // The blocking texture uploads finished long ago, but draws from the last
    // in-flight frame may still reference the SRVs.
    m_pso.Reset();
    m_rootSig.Reset();
    m_albedo.Reset();
    m_normal.Reset();
    m_orm.Reset();
    m_srvs.Reset();
    m_shaders.Shutdown();
    m_objects.clear();
    m_ready = false;
}

void NormalMapScene::OnUpdate(const DemoContext& ctx) {
    m_spinAngle += ctx.dt * 0.6f;
    if (m_animateLight) {
        m_time += ctx.dt;
        m_lightAzimuth = 0.8f + m_time * 0.3f;
    }
}

void NormalMapScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;

    cmd->SetGraphicsRootSignature(m_rootSig.Get());
    cmd->SetPipelineState(m_pso.Get());
    cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    m_srvs.BindTable(cmd, 2);

    FrameCB fcb = {};
    XMStoreFloat4(&fcb.LightDir, ComputeLightDir(m_lightElevation, m_lightAzimuth));
    fcb.CameraPos      = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2], 1.0f };
    fcb.NormalStrength = m_normalStrength;
    fcb.UseNormalMap   = m_useNormalMap ? 1 : 0;
    fcb.FlipGreen      = m_flipGreen ? 1 : 0;
    fcb.ViewMode       = m_viewMode;
    fcb.GammaCorrect   = m_gammaCorrect ? 1 : 0;
    fcb.HeightScale    = m_heightScale;
    fcb.UsePom         = m_usePom ? 1 : 0;
    fcb.PomShadow      = m_pomShadow ? 1 : 0;
    fcb.PomSteps       = m_pomSteps;
    fcb.RoughnessScale = m_roughnessScale;
    fcb.UseMaterialMap = m_useMaterialMap ? 1 : 0;
    if (!ctx.objectCB->BindCbv(cmd, 1, fcb)) return;

    const XMMATRIX camVP = ctx.camera->GetViewProjection();
    for (const Object& o : m_objects) {
        XMMATRIX model = XMMatrixScaling(o.Scale.x, o.Scale.y, o.Scale.z);
        if (o.Spin)
            model *= XMMatrixRotationRollPitchYaw(m_spinAngle * 0.5f, m_spinAngle, 0.0f);
        model *= XMMatrixTranslation(o.Pos.x, o.Pos.y, o.Pos.z);

        ObjCB cb;
        XMStoreFloat4x4(&cb.MVP, model * camVP);
        XMStoreFloat4x4(&cb.Model, model);
        cb.UvTiling = { o.Tiling.x * m_tilingScale, o.Tiling.y * m_tilingScale, 0, 0 };
        cb.Material = { o.RoughMult, o.Metallic, 0, 0 };
        if (!ctx.objectCB->BindCbv(cmd, 0, cb)) continue;
        m_cube->Draw(cmd);
    }
}

void NormalMapScene::OnImGui() {
    ImGui::TextDisabled("Textures: %s (%ux%u, %u mips)",
                        m_fromFiles ? "Assets/Brick*.png" : "procedural brick",
                        m_albedo.Width(), m_albedo.Height(), m_albedo.MipLevels());
    ImGui::Separator();

    ImGui::Checkbox("Normal mapping", &m_useNormalMap);
    ImGui::SliderFloat("Strength", &m_normalStrength, 0.0f, 3.0f, "%.2f");
    ImGui::Checkbox("Flip green (OpenGL-style map)", &m_flipGreen);
    ImGui::Checkbox("Gamma-correct output", &m_gammaCorrect);
    ImGui::SliderFloat("UV tiling scale", &m_tilingScale, 0.25f, 4.0f, "%.2f");

    ImGui::Separator();
    ImGui::Checkbox("Parallax occlusion", &m_usePom);
    ImGui::SliderFloat("Height scale", &m_heightScale, 0.0f, 0.15f, "%.3f");
    ImGui::SliderInt("POM max steps", &m_pomSteps, 8, 64);
    ImGui::Checkbox("Contact shadows", &m_pomShadow);

    ImGui::Separator();
    ImGui::Checkbox("ORM material map", &m_useMaterialMap);
    ImGui::SliderFloat("Roughness scale", &m_roughnessScale, 0.25f, 2.0f, "%.2f");
    ImGui::TextDisabled("Cook-Torrance GGX; the tall box is forced metallic.");

    const char* modes[] = { "Lit", "Albedo only", "Geometric normals", "Mapped normals",
                            "Height", "Material (ORM)" };
    ImGui::Combo("View", &m_viewMode, modes, 6);

    ImGui::Separator();
    ImGui::Checkbox("Animate light", &m_animateLight);
    ImGui::SliderFloat("Light elevation", &m_lightElevation, 15.0f, 80.0f, "%.0f deg");
    ImGui::SliderFloat("Light azimuth",   &m_lightAzimuth, 0.0f, 6.28f, "%.2f");

    ImGui::Separator();
    ImGui::TextDisabled("Albedo samples through an _SRGB SRV (decoded to\n"
                        "linear by the hardware); the normal map stays UNORM —\n"
                        "vectors are data, not color. Mips are CPU-box-filtered\n"
                        "(sRGB-aware for albedo).");
}
