#include "Scenes/OitScene.h"

#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/DX12/RootSignatureBuilder.h"
#include "Core/Camera.h"

#include "imgui.h"
#include <DirectXMath.h>
#include <cmath>

using namespace SGE;
using namespace DirectX;

namespace {
struct ObjCB { XMFLOAT4X4 mvp; XMFLOAT4X4 model; XMFLOAT4X4 modelView; XMFLOAT4 color; };

// RT0 additive (sum premult*weight), RT1 multiplicative (dst *= 1-alpha).
D3D12_BLEND_DESC AccumBlend() {
    D3D12_BLEND_DESC b = {};
    b.IndependentBlendEnable = TRUE;
    auto& r0 = b.RenderTarget[0];
    r0.BlendEnable = TRUE;
    r0.SrcBlend = D3D12_BLEND_ONE; r0.DestBlend = D3D12_BLEND_ONE; r0.BlendOp = D3D12_BLEND_OP_ADD;
    r0.SrcBlendAlpha = D3D12_BLEND_ONE; r0.DestBlendAlpha = D3D12_BLEND_ONE; r0.BlendOpAlpha = D3D12_BLEND_OP_ADD;
    r0.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    auto& r1 = b.RenderTarget[1];
    r1.BlendEnable = TRUE;
    r1.SrcBlend = D3D12_BLEND_ZERO; r1.DestBlend = D3D12_BLEND_INV_SRC_COLOR; r1.BlendOp = D3D12_BLEND_OP_ADD;
    r1.SrcBlendAlpha = D3D12_BLEND_ZERO; r1.DestBlendAlpha = D3D12_BLEND_INV_SRC_ALPHA; r1.BlendOpAlpha = D3D12_BLEND_OP_ADD;
    r1.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_RED;
    return b;
}
// final = avg*(1-reveal) + opaque*reveal   (src.a = revealage)
D3D12_BLEND_DESC CompositeBlend() {
    D3D12_BLEND_DESC b = {};
    auto& r = b.RenderTarget[0];
    r.BlendEnable = TRUE;
    r.SrcBlend = D3D12_BLEND_INV_SRC_ALPHA; r.DestBlend = D3D12_BLEND_SRC_ALPHA; r.BlendOp = D3D12_BLEND_OP_ADD;
    r.SrcBlendAlpha = D3D12_BLEND_ONE; r.DestBlendAlpha = D3D12_BLEND_ZERO; r.BlendOpAlpha = D3D12_BLEND_OP_ADD;
    r.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    return b;
}
// Standard (order-dependent) alpha blend for the naive comparison.
D3D12_BLEND_DESC AlphaBlend() {
    D3D12_BLEND_DESC b = {};
    auto& r = b.RenderTarget[0];
    r.BlendEnable = TRUE;
    r.SrcBlend = D3D12_BLEND_SRC_ALPHA; r.DestBlend = D3D12_BLEND_INV_SRC_ALPHA; r.BlendOp = D3D12_BLEND_OP_ADD;
    r.SrcBlendAlpha = D3D12_BLEND_ONE; r.DestBlendAlpha = D3D12_BLEND_INV_SRC_ALPHA; r.BlendOpAlpha = D3D12_BLEND_OP_ADD;
    r.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
    return b;
}

void SetTargets(ID3D12GraphicsCommandList* c, const D3D12_CPU_DESCRIPTOR_HANDLE* rtvs, UINT n,
                const D3D12_CPU_DESCRIPTOR_HANDLE* dsv, uint32_t w, uint32_t h) {
    c->OMSetRenderTargets(n, rtvs, FALSE, dsv);
    D3D12_VIEWPORT vp = { 0, 0, float(w), float(h), 0, 1 };
    D3D12_RECT     sc = { 0, 0, LONG(w), LONG(h) };
    c->RSSetViewports(1, &vp); c->RSSetScissorRects(1, &sc);
}
} // namespace

const char* OitScene::Description() const {
    return "Order-independent transparency (weighted-blended OIT). Overlapping "
           "translucent spheres are accumulated into additive-colour + "
           "multiplicative-revealage targets with depth-weighted blending, so "
           "they composite correctly in ANY order (no sorting). A composite pass "
           "resolves them over the opaque scene. Toggle to naive alpha blending "
           "and orbit the camera to see the overlaps flip — the problem OIT solves.";
}

void OitScene::OnLoad(const DemoContext& ctx) {
    m_shaders.Initialize(L"Shaders");
    if (!BuildPipelines(ctx)) return;
    m_srvs.Create(ctx.device, 2);
    if (!CreateTargets(ctx, ctx.renderer->GetWidth(), ctx.renderer->GetHeight())) return;
    m_ready = true;
}

void OitScene::OnUnload() {
    m_ready = false;
    m_accum.Reset(); m_reveal.Reset(); m_srvs.Reset();
    m_opaquePSO.Reset(); m_accumPSO.Reset(); m_naivePSO.Reset(); m_compositePSO.Reset();
    m_objRS.Reset(); m_compositeRS.Reset();
    m_shaders.Shutdown();
}

bool OitScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* dev = ctx.device;
    if (!RootSignatureBuilder().Cbv(0).Build(dev, m_objRS)) return false;

    // Opaque.
    {
        auto vs = m_shaders.GetOrCompile(L"OitOpaque.hlsl", "VSMain", "vs_6_0");
        auto ps = m_shaders.GetOrCompile(L"OitOpaque.hlsl", "PSMain", "ps_6_0");
        if (!vs || !ps) return false;
        GraphicsPipelineDesc d; d.rootSignature = m_objRS.Get(); d.vs = vs; d.ps = ps;
        d.depthEnable = true; d.dsvFormat = DXGI_FORMAT_D32_FLOAT;
        if (!m_opaquePSO.Create(dev, d)) return false;
    }
    // Transparent WBOIT accum (MRT, independent blend, depth test no write, cull none).
    {
        auto vs = m_shaders.GetOrCompile(L"OitTransparent.hlsl", "VSMain",  "vs_6_0");
        auto ps = m_shaders.GetOrCompile(L"OitTransparent.hlsl", "PSAccum", "ps_6_0");
        if (!vs || !ps) return false;
        GraphicsPipelineDesc d; d.rootSignature = m_objRS.Get(); d.vs = vs; d.ps = ps;
        d.depthEnable = true; d.depthWrite = false; d.dsvFormat = DXGI_FORMAT_D32_FLOAT;
        d.cullMode = D3D12_CULL_MODE_NONE;
        d.rtvCount = 2;
        d.rtvFormats[0] = DXGI_FORMAT_R16G16B16A16_FLOAT;
        d.rtvFormats[1] = DXGI_FORMAT_R16_FLOAT;
        d.blendEnable = true; d.blend = AccumBlend();
        if (!m_accumPSO.Create(dev, d)) return false;
    }
    // Transparent naive alpha (to back buffer, order-dependent).
    {
        auto vs = m_shaders.GetOrCompile(L"OitTransparent.hlsl", "VSMain",  "vs_6_0");
        auto ps = m_shaders.GetOrCompile(L"OitTransparent.hlsl", "PSNaive", "ps_6_0");
        if (!vs || !ps) return false;
        GraphicsPipelineDesc d; d.rootSignature = m_objRS.Get(); d.vs = vs; d.ps = ps;
        d.depthEnable = true; d.depthWrite = false; d.dsvFormat = DXGI_FORMAT_D32_FLOAT;
        d.cullMode = D3D12_CULL_MODE_NONE;
        d.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
        d.blendEnable = true; d.blend = AlphaBlend();
        if (!m_naivePSO.Create(dev, d)) return false;
    }
    // Composite.
    if (!RootSignatureBuilder().SrvTable(0, 2).SamplerLinearClamp(0).Build(dev, m_compositeRS))
        return false;
    {
        auto vs = m_shaders.GetOrCompile(L"OitComposite.hlsl", "VSMain", "vs_6_0");
        auto ps = m_shaders.GetOrCompile(L"OitComposite.hlsl", "PSMain", "ps_6_0");
        if (!vs || !ps) return false;
        GraphicsPipelineDesc d; d.rootSignature = m_compositeRS.Get(); d.vs = vs; d.ps = ps;
        d.depthEnable = false; d.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
        d.blendEnable = true; d.blend = CompositeBlend();
        if (!m_compositePSO.Create(dev, d)) return false;
    }
    return true;
}

bool OitScene::CreateTargets(const DemoContext& ctx, uint32_t w, uint32_t h) {
    m_w = w; m_h = h;
    const float zero[4] = { 0, 0, 0, 0 };
    const float one[4]  = { 1, 0, 0, 0 };
    if (!m_accum.Create(ctx.device, w, h, DXGI_FORMAT_R16G16B16A16_FLOAT, zero)) return false;
    if (!m_reveal.Create(ctx.device, w, h, DXGI_FORMAT_R16_FLOAT, one)) return false;
    m_accum.CreateSrvInto(ctx.device, m_srvs.Cpu(0));
    m_reveal.CreateSrvInto(ctx.device, m_srvs.Cpu(1));
    m_graph.ResetStateCache();
    return true;
}

void OitScene::OnUpdate(const DemoContext& ctx) { if (m_spin) m_time += ctx.dt; }

void OitScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    const uint32_t w = r->GetWidth(), h = r->GetHeight();
    if (w != m_w || h != m_h) { r->WaitForGPU(); CreateTargets(ctx, w, h); }

    const XMMATRIX view     = ctx.camera->GetView();
    const XMMATRIX viewProj = ctx.camera->GetViewProjection();
    const XMMATRIX spin     = XMMatrixRotationY(m_time * 0.4f);

    auto draw = [&](ID3D12GraphicsCommandList* c, Mesh* m, XMMATRIX model, XMFLOAT4 color) {
        ObjCB o; XMStoreFloat4x4(&o.mvp, model * viewProj);
        XMStoreFloat4x4(&o.model, model); XMStoreFloat4x4(&o.modelView, model * view);
        o.color = color;
        ctx.objectCB->BindCbv(c, 0, o); m->Draw(c);
    };
    // Opaque geometry: a ground slab + a small centre cube inside the cluster.
    auto drawOpaque = [&](ID3D12GraphicsCommandList* c) {
        draw(c, m_cube, XMMatrixScaling(24.0f, 0.3f, 24.0f) * XMMatrixTranslation(0, -2.0f, 0),
             { 0.55f, 0.55f, 0.58f, 1.0f });
        draw(c, m_cube, XMMatrixScaling(0.9f, 0.9f, 0.9f) * spin, { 0.85f, 0.8f, 0.4f, 1.0f });
    };
    // Overlapping translucent spheres (a little cluster that intersects itself).
    const float cols[5][3] = { {0.95f,0.3f,0.3f},{0.3f,0.9f,0.4f},{0.35f,0.5f,0.95f},
                               {0.95f,0.85f,0.3f},{0.8f,0.4f,0.9f} };
    auto drawTransparent = [&](ID3D12GraphicsCommandList* c) {
        for (int k = 0; k < 5; ++k) {
            const float a = XM_2PI * float(k) / 5.0f;
            const XMMATRIX m = XMMatrixScaling(1.3f, 1.3f, 1.3f) *
                XMMatrixTranslation(cosf(a) * 1.1f, sinf(a) * 0.8f, sinf(a * 1.3f) * 0.6f) * spin;
            draw(c, m_sphere, m, { cols[k][0], cols[k][1], cols[k][2], m_alpha });
        }
    };

    m_graph.Begin(cmd);
    m_graph.SetProfiler(&r->GetProfiler());
    const RgHandle back  = m_graph.Import("BackBuffer", r->BackBufferResource(), RgState::RenderTarget);
    const RgHandle depth = m_graph.Import("SceneDepth", r->DepthResource(),      RgState::DepthWrite);
    const RgHandle accum = m_graph.Import("Accum",      m_accum.Resource(),      RgState::Common);
    const RgHandle rev   = m_graph.Import("Revealage",  m_reveal.Resource(),     RgState::Common);

    // ---- Opaque -> back buffer (+ depth) ----
    m_graph.AddPass("Opaque",
        { Write(back, RgState::RenderTarget), Write(depth, RgState::DepthWrite) },
        [this, r, drawOpaque](ID3D12GraphicsCommandList* c) {
            r->BindBackBufferTargets(c);
            c->SetGraphicsRootSignature(m_objRS.Get());
            c->SetPipelineState(m_opaquePSO.Get());
            c->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            drawOpaque(c);
        });

    if (m_wboit) {
        // ---- Accumulate transparency (depth tested against opaque, no write) ----
        m_graph.AddPass("OIT Accum",
            { Write(accum, RgState::RenderTarget), Write(rev, RgState::RenderTarget) },
            [this, r, drawTransparent](ID3D12GraphicsCommandList* c) {
                D3D12_CPU_DESCRIPTOR_HANDLE rtvs[2] = { m_accum.Rtv(), m_reveal.Rtv() };
                D3D12_CPU_DESCRIPTOR_HANDLE dsv = r->GetDepthDSV();
                SetTargets(c, rtvs, 2, &dsv, m_w, m_h);
                m_accum.ClearRtv(c); m_reveal.ClearRtv(c);
                c->SetGraphicsRootSignature(m_objRS.Get());
                c->SetPipelineState(m_accumPSO.Get());
                c->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
                drawTransparent(c);
            });
        // ---- Composite over the opaque image ----
        m_graph.AddPass("OIT Composite",
            { Read(accum, RgState::PixelShader), Read(rev, RgState::PixelShader),
              Write(back, RgState::RenderTarget) },
            [this, r](ID3D12GraphicsCommandList* c) {
                D3D12_CPU_DESCRIPTOR_HANDLE rtv = r->GetBackBufferRTV();
                SetTargets(c, &rtv, 1, nullptr, m_w, m_h);
                c->SetGraphicsRootSignature(m_compositeRS.Get());
                c->SetPipelineState(m_compositePSO.Get());
                m_srvs.BindTable(c, 0, 0);
                c->IASetVertexBuffers(0, 0, nullptr);
                c->DrawInstanced(3, 1, 0, 0);
            },
            /*sideEffect*/ true);
    } else {
        // ---- Naive: alpha blend straight to the back buffer (order-dependent) ----
        m_graph.AddPass("Naive Transparency",
            { Write(back, RgState::RenderTarget) },
            [this, r, drawTransparent](ID3D12GraphicsCommandList* c) {
                D3D12_CPU_DESCRIPTOR_HANDLE rtv = r->GetBackBufferRTV();
                D3D12_CPU_DESCRIPTOR_HANDLE dsv = r->GetDepthDSV();
                SetTargets(c, &rtv, 1, &dsv, m_w, m_h);
                c->SetGraphicsRootSignature(m_objRS.Get());
                c->SetPipelineState(m_naivePSO.Get());
                c->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
                drawTransparent(c);
            },
            /*sideEffect*/ true);
    }

    m_graph.Execute();
}

void OitScene::OnImGui() {
    ImGui::Checkbox("Weighted-blended OIT", &m_wboit);
    ImGui::TextDisabled(m_wboit ? "Order-INdependent: correct at any angle."
                                : "Naive alpha blend: overlaps flip as you orbit.");
    ImGui::SliderFloat("Alpha", &m_alpha, 0.1f, 0.9f, "%.2f");
    ImGui::Checkbox("Spin", &m_spin);
    ImGui::Separator();
    ImGui::TextDisabled("5 intersecting translucent spheres + an opaque cube.");
}

bool OitScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 0.0f; pos[1] = 1.0f; pos[2] = -6.5f;
    yaw = 0.0f; pitch = -0.08f;
    return true;
}
