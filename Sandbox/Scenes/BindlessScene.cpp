#include "Scenes/BindlessScene.h"

#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/ImageLoader.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/DX12/RootSignatureBuilder.h"
#include "Core/Camera.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <cmath>

using namespace SGE;
using namespace DirectX;

namespace {
struct ObjCB {
    XMFLOAT4X4 mvp;
    XMFLOAT4X4 model;
    uint32_t   texIndex;
    uint32_t   pad[3];
};
constexpr int kTextureCount = 16;

// A distinct 2-colour checker per index, hues spread around the wheel, so every
// cube in the grid is visibly its own texture.
Image MakeColorChecker(int i) {
    auto hsv = [](float h, float s, float v, uint8_t out[4]) {
        const float c = v * s, x = c * (1.0f - std::fabs(std::fmod(h / 60.0f, 2.0f) - 1.0f)), m = v - c;
        float r = 0, g = 0, b = 0;
        if      (h <  60) { r = c; g = x; }
        else if (h < 120) { r = x; g = c; }
        else if (h < 180) { g = c; b = x; }
        else if (h < 240) { g = x; b = c; }
        else if (h < 300) { r = x; b = c; }
        else              { r = c; b = x; }
        out[0] = uint8_t((r + m) * 255); out[1] = uint8_t((g + m) * 255);
        out[2] = uint8_t((b + m) * 255); out[3] = 255;
    };
    const float h = 360.0f * float(i) / float(kTextureCount);
    uint8_t a[4], b[4];
    hsv(h, 0.65f, 0.95f, a);
    hsv(std::fmod(h + 25.0f, 360.0f), 0.45f, 0.55f, b);
    return MakeCheckerboard(128, 4, a, b);
}
} // namespace

const char* BindlessScene::Description() const {
    return "Bindless rendering (Shader Model 6.6). A grid of cubes, each with a "
           "different texture, is drawn with ONE root signature and NO per-draw "
           "descriptor table. All textures sit in a single shader-visible heap; "
           "each cube passes only an integer index, and the pixel shader fetches "
           "its texture with ResourceDescriptorHeap[index]. This is the "
           "foundation for GPU-driven rendering and ray tracing.";
}

void BindlessScene::OnLoad(const DemoContext& ctx) {
    m_supported = ctx.renderer->SupportsBindless();
    if (!m_supported)
        return;

    m_shaders.Initialize(L"Shaders");

    // One root CBV (b0) + a static sampler, flagged so the shader may index the
    // heap directly. No SRV table at all — that's the point.
    if (!RootSignatureBuilder().Cbv(0).SamplerAnisoWrap(0).DirectlyIndexed()
             .Build(ctx.device, m_rootSig))
        return;

    auto vs = m_shaders.GetOrCompile(L"Bindless.hlsl", "VSMain", "vs_6_6");
    auto ps = m_shaders.GetOrCompile(L"Bindless.hlsl", "PSMain", "ps_6_6");
    if (!vs || !ps)
        return;
    GraphicsPipelineDesc desc;
    desc.rootSignature = m_rootSig.Get();
    desc.vs = vs; desc.ps = ps;
    desc.depthEnable = true;
    desc.dsvFormat   = DXGI_FORMAT_D32_FLOAT;
    if (!m_pso.Create(ctx.device, desc))
        return;

    // The bindless heap, and a spread of textures registered into it. Each
    // texture's SRV is written at its allocated index; that index is all the
    // shader ever needs.
    if (!m_bindless.Create(ctx.device, 256))
        return;
    m_textures.resize(kTextureCount);
    m_texIndex.resize(kTextureCount);
    for (int i = 0; i < kTextureCount; ++i) {
        m_textures[i].CreateFromImage(ctx.device, ctx.renderer->GetCommandQueue(),
                                      MakeColorChecker(i), /*srgb*/ true);
        m_texIndex[i] = m_bindless.Allocate();
        m_textures[i].CreateSrvInto(ctx.device, m_bindless.Cpu(m_texIndex[i]));
    }
    m_ready = true;
}

void BindlessScene::OnUnload() {
    m_ready = false;
    for (auto& t : m_textures) t.Reset();
    m_textures.clear();
    m_texIndex.clear();
    m_bindless.Reset();
    m_pso.Reset();
    m_rootSig.Reset();
    if (m_supported)
        m_shaders.Shutdown();
}

void BindlessScene::OnUpdate(const DemoContext& ctx) {
    m_time += ctx.dt * m_spinSpeed;
}

void BindlessScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;

    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;
    const XMMATRIX viewProj = ctx.camera->GetViewProjection();

    m_graph.Begin(cmd);
    m_graph.SetProfiler(&r->GetProfiler());
    const RgHandle back  = m_graph.Import("BackBuffer", r->BackBufferResource(), RgState::RenderTarget);
    const RgHandle depth = m_graph.Import("SceneDepth", r->DepthResource(),      RgState::DepthWrite);

    m_graph.AddPass("Bindless",
        { Write(back, RgState::RenderTarget), Write(depth, RgState::DepthWrite) },
        [this, r, viewProj, &ctx](ID3D12GraphicsCommandList* c) {
            r->BindBackBufferTargets(c);
            m_bindless.Bind(c);                 // the heap ResourceDescriptorHeap[] resolves against
            c->SetGraphicsRootSignature(m_rootSig.Get());
            c->SetPipelineState(m_pso.Get());
            c->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

            const int   n       = m_gridN;
            const float spacing = 2.2f;
            const float offset  = (n - 1) * 0.5f * spacing;
            int k = 0;
            for (int gy = 0; gy < n; ++gy) {
                for (int gx = 0; gx < n; ++gx, ++k) {
                    const XMMATRIX model =
                        XMMatrixRotationY(m_time + k * 0.3f) *
                        XMMatrixTranslation(gx * spacing - offset, gy * spacing - offset, 0.0f);
                    ObjCB o = {};
                    XMStoreFloat4x4(&o.mvp,   model * viewProj);
                    XMStoreFloat4x4(&o.model, model);
                    o.texIndex = m_texIndex[k % kTextureCount]; // just an integer — no table bind
                    if (ctx.objectCB->BindCbv(c, 0, o))
                        m_cube->Draw(c);
                }
            }
        },
        /*sideEffect*/ true);

    m_graph.Execute();
}

void BindlessScene::OnImGui() {
    if (!m_supported) {
        ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.3f, 1),
                           "Bindless unsupported: GPU/driver lacks SM 6.6 + resource-binding tier 3.");
        return;
    }
    ImGui::SliderInt("Grid size", &m_gridN, 2, 12);
    ImGui::SliderFloat("Spin speed", &m_spinSpeed, 0.0f, 2.0f, "%.2f");
    ImGui::Separator();
    ImGui::Text("%d cubes, %d textures", m_gridN * m_gridN, kTextureCount);
    ImGui::TextDisabled("One root signature, one heap bind, zero per-draw tables.");
    ImGui::TextDisabled("Each cube passes only ResourceDescriptorHeap index %u..%u.",
                        m_texIndex.empty() ? 0 : m_texIndex.front(),
                        m_texIndex.empty() ? 0 : m_texIndex.back());
}

bool BindlessScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 0.0f; pos[1] = 0.0f; pos[2] = -16.0f;
    yaw = 0.0f; pitch = 0.0f;
    return true;
}
