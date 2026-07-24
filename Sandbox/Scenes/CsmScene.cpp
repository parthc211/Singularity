#include "Scenes/CsmScene.h"

#include "Core/Camera.h"
#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"

#include "imgui.h"
#include <random>
#include <cmath>
#include <cstring>
#include <algorithm>

using namespace SGE;
using namespace DirectX;

namespace {
struct ShadowObjCB { XMFLOAT4X4 LightMVP; };
struct LitObjCB    { XMFLOAT4X4 MVP; XMFLOAT4X4 Model; XMFLOAT4 BaseColor; };
struct LitFrameCB  {
    XMFLOAT4X4 CascadeVP[4];
    XMFLOAT4   CascadeSplits;
    XMFLOAT4   LightDir;
    XMFLOAT4   CameraPos;
    float Bias; float TexelSize; int CascadeCount; int DebugTint;
};
}

const char* CsmScene::Description() const {
    return "Cascaded shadow maps. The view frustum is split into 4 depth ranges; "
           "each gets a tight light-space depth map (stable, texel-snapped) stored "
           "as one array slice. The main pass picks a cascade per pixel by distance. "
           "Enable 'tint by cascade' to see the split bands.";
}

XMVECTOR CsmScene::LightDir() const {
    const float el = XMConvertToRadians(m_lightElevation);
    return XMVector3Normalize(XMVectorSet(cosf(el) * cosf(m_lightAzimuth), -sinf(el),
                                          cosf(el) * sinf(m_lightAzimuth), 0.0f));
}

void CsmScene::ComputeCascades(const Camera& camera, XMFLOAT4X4 outVP[4], float outSplits[4]) const {
    XMFLOAT4X4 p; XMStoreFloat4x4(&p, camera.GetProjection());
    const float tanX = 1.0f / p._11;
    const float tanY = 1.0f / p._22;
    const XMMATRIX invView = XMMatrixInverse(nullptr, camera.GetView());
    const XMVECTOR L = LightDir();

    // Practical split scheme: blend uniform and logarithmic splits.
    const float nearD = 0.5f, farD = m_shadowFar;
    float splitFar[4];
    for (int i = 0; i < kCascades; ++i) {
        const float pi  = float(i + 1) / kCascades;
        const float uni = nearD + (farD - nearD) * pi;
        const float lg  = nearD * powf(farD / nearD, pi);
        splitFar[i] = m_lambda * lg + (1.0f - m_lambda) * uni;
        outSplits[i] = splitFar[i];
    }

    float prev = nearD;
    for (int c = 0; c < kCascades; ++c) {
        const float n = prev, f = splitFar[c];
        prev = f;

        // 8 world-space corners of this frustum slice.
        XMVECTOR corners[8]; int idx = 0;
        for (float d : { n, f })
            for (float sy : { -1.0f, 1.0f })
                for (float sx : { -1.0f, 1.0f }) {
                    XMVECTOR v = XMVectorSet(sx * d * tanX, sy * d * tanY, d, 1.0f);
                    corners[idx++] = XMVector3TransformCoord(v, invView);
                }

        XMVECTOR center = XMVectorZero();
        for (auto& cr : corners) center = XMVectorAdd(center, cr);
        center = XMVectorScale(center, 1.0f / 8.0f);

        float radius = 0.0f;
        for (auto& cr : corners)
            radius = std::max(radius, XMVectorGetX(XMVector3Length(XMVectorSubtract(cr, center))));
        radius = ceilf(radius); // stabilize tiny variations

        // Light basis, then snap the centre to the shadow texel grid (kills shimmer).
        const XMVECTOR up0   = XMVectorSet(0, 1, 0, 0);
        const XMVECTOR right = XMVector3Normalize(XMVector3Cross(up0, L));
        const XMVECTOR up    = XMVector3Normalize(XMVector3Cross(L, right));
        const float texel = (2.0f * radius) / kShadowSize;
        float cr2 = XMVectorGetX(XMVector3Dot(center, right));
        float cu2 = XMVectorGetX(XMVector3Dot(center, up));
        float cl2 = XMVectorGetX(XMVector3Dot(center, L));
        cr2 = floorf(cr2 / texel + 0.5f) * texel;
        cu2 = floorf(cu2 / texel + 0.5f) * texel;
        center = XMVectorAdd(XMVectorAdd(XMVectorScale(right, cr2), XMVectorScale(up, cu2)),
                             XMVectorScale(L, cl2));

        const XMVECTOR eye = XMVectorSubtract(center, XMVectorScale(L, radius * 2.0f));
        const XMMATRIX view = XMMatrixLookAtLH(eye, center, up);
        const XMMATRIX proj = XMMatrixOrthographicLH(2.0f * radius, 2.0f * radius, 0.0f, radius * 4.0f);
        XMStoreFloat4x4(&outVP[c], view * proj); // no transpose (engine convention)
    }
}

void CsmScene::BuildObjects() {
    m_objects.clear();
    auto add = [&](XMMATRIX m, XMFLOAT4 c) {
        Object o; XMStoreFloat4x4(&o.Model, m); o.Color = c; m_objects.push_back(o);
    };

    add(XMMatrixScaling(200, 1, 200) * XMMatrixTranslation(0, -0.5f, 0), { 0.55f, 0.57f, 0.55f, 1 }); // ground

    std::mt19937 rng(7);
    std::uniform_real_distribution<float> pos(-80.0f, 80.0f), hgt(1.5f, 8.0f),
                                          wid(1.0f, 3.0f), col(0.4f, 0.95f);
    for (int i = 0; i < 36; ++i) {
        const float sx = wid(rng), sy = hgt(rng), sz = wid(rng);
        const float x = pos(rng), z = pos(rng);
        add(XMMatrixScaling(sx, sy, sz) * XMMatrixTranslation(x, sy * 0.5f, z),
            { col(rng), col(rng), col(rng), 1 });
    }
}

bool CsmScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* device = ctx.device;
    m_shaders.Initialize(L"Shaders");

    // Shadow pass: vertex-only depth, one CBV (the cascade light MVP).
    D3D12_ROOT_PARAMETER sp      = {};
    sp.ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    sp.Descriptor.ShaderRegister = 0;
    sp.ShaderVisibility          = D3D12_SHADER_VISIBILITY_VERTEX;
    if (!m_shadowRootSig.Create(device, &sp, 1)) return false;

    auto svs = m_shaders.GetOrCompile(L"ShadowDepth.hlsl", "VSMain", "vs_6_0");
    if (!svs) return false;
    GraphicsPipelineDesc sd;
    sd.rootSignature = m_shadowRootSig.Get();
    sd.vs = svs; sd.ps = nullptr;
    sd.rtvCount = 0; sd.depthEnable = true; sd.dsvFormat = DXGI_FORMAT_D32_FLOAT;
    if (!m_shadowPSO.Create(device, sd)) return false;

    // Main pass: b0 obj, b1 frame, array SRV table, comparison sampler.
    D3D12_DESCRIPTOR_RANGE range = {};
    range.RangeType                         = D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
    range.NumDescriptors                    = 1;
    range.BaseShaderRegister                = 0;
    range.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;
    D3D12_ROOT_PARAMETER lp[3] = {};
    lp[0].ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    lp[0].Descriptor.ShaderRegister = 0; lp[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    lp[1].ParameterType             = D3D12_ROOT_PARAMETER_TYPE_CBV;
    lp[1].Descriptor.ShaderRegister = 1; lp[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
    lp[2].ParameterType                       = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
    lp[2].DescriptorTable.NumDescriptorRanges = 1;
    lp[2].DescriptorTable.pDescriptorRanges   = &range;
    lp[2].ShaderVisibility                    = D3D12_SHADER_VISIBILITY_PIXEL;

    D3D12_STATIC_SAMPLER_DESC samp = {};
    samp.Filter           = D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT;
    samp.AddressU = samp.AddressV = samp.AddressW = D3D12_TEXTURE_ADDRESS_MODE_BORDER;
    samp.BorderColor      = D3D12_STATIC_BORDER_COLOR_OPAQUE_WHITE;
    samp.ComparisonFunc   = D3D12_COMPARISON_FUNC_LESS_EQUAL;
    samp.ShaderRegister   = 0; samp.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
    samp.MaxLOD           = D3D12_FLOAT32_MAX;
    if (!m_litRootSig.Create(device, lp, 3, &samp, 1)) return false;

    auto lvs = m_shaders.GetOrCompile(L"ShadowLitCsm.hlsl", "VSMain", "vs_6_0");
    auto lps = m_shaders.GetOrCompile(L"ShadowLitCsm.hlsl", "PSMain", "ps_6_0");
    if (!lvs || !lps) return false;
    GraphicsPipelineDesc ld;
    ld.rootSignature = m_litRootSig.Get();
    ld.vs = lvs; ld.ps = lps;
    ld.depthEnable = true; ld.rtvFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
    return m_litPSO.Create(device, ld);
}

void CsmScene::OnLoad(const DemoContext& ctx) {
    BuildObjects();
    const bool cm = m_csm.Create(ctx.device, kShadowSize, kCascades);
    const bool pl = BuildPipelines(ctx);
    m_ready = cm && pl;
}

void CsmScene::OnUnload() {
    m_csm.Reset();
    m_shadowPSO.Reset(); m_litPSO.Reset();
    m_shadowRootSig.Reset(); m_litRootSig.Reset();
    m_shaders.Shutdown();
    m_objects.clear();
    m_ready = false;
}

void CsmScene::OnUpdate(const DemoContext& ctx) {
    if (m_animateLight) { m_time += ctx.dt; m_lightAzimuth = 0.7f + m_time * 0.15f; }
}

void CsmScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;
    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    XMFLOAT4X4 cascadeVP[4]; float splits[4];
    ComputeCascades(*ctx.camera, cascadeVP, splits);
    const XMMATRIX camVP = ctx.camera->GetViewProjection();

    // ---- Shadow pass: render each cascade slice. ----
    m_csm.BeginCascadePass(cmd);
    cmd->SetGraphicsRootSignature(m_shadowRootSig.Get());
    cmd->SetPipelineState(m_shadowPSO.Get());
    cmd->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    for (int c = 0; c < kCascades; ++c) {
        m_csm.BindCascade(cmd, c);
        const XMMATRIX vp = XMLoadFloat4x4(&cascadeVP[c]);
        for (const Object& o : m_objects) {
            ShadowObjCB cb;
            XMStoreFloat4x4(&cb.LightMVP, XMLoadFloat4x4(&o.Model) * vp);
            auto a = ctx.objectCB->Allocate(sizeof(cb));
            if (!a.Cpu) continue;
            std::memcpy(a.Cpu, &cb, sizeof(cb));
            cmd->SetGraphicsRootConstantBufferView(0, a.Gpu);
            m_cube->Draw(cmd);
        }
    }
    m_csm.EndCascadePass(cmd);

    // ---- Restore back buffer + depth + full viewport. ----
    D3D12_CPU_DESCRIPTOR_HANDLE backRtv = r->GetBackBufferRTV();
    D3D12_CPU_DESCRIPTOR_HANDLE dsv     = r->GetDepthDSV();
    cmd->OMSetRenderTargets(1, &backRtv, FALSE, &dsv);
    D3D12_VIEWPORT vp = { 0, 0, float(r->GetWidth()), float(r->GetHeight()), 0.0f, 1.0f };
    D3D12_RECT     sc = { 0, 0, LONG(r->GetWidth()), LONG(r->GetHeight()) };
    cmd->RSSetViewports(1, &vp);
    cmd->RSSetScissorRects(1, &sc);

    // ---- Main pass: lit + cascaded shadows. ----
    cmd->SetGraphicsRootSignature(m_litRootSig.Get());
    cmd->SetPipelineState(m_litPSO.Get());
    ID3D12DescriptorHeap* heaps[] = { m_csm.SrvHeap() };
    cmd->SetDescriptorHeaps(1, heaps);
    cmd->SetGraphicsRootDescriptorTable(2, m_csm.Srv());

    LitFrameCB fcb{};
    for (int c = 0; c < kCascades; ++c) fcb.CascadeVP[c] = cascadeVP[c];
    fcb.CascadeSplits = { splits[0], splits[1], splits[2], splits[3] };
    XMStoreFloat4(&fcb.LightDir, LightDir());
    fcb.CameraPos    = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2], 1.0f };
    fcb.Bias         = m_bias;
    fcb.TexelSize    = 1.0f / kShadowSize;
    fcb.CascadeCount = kCascades;
    fcb.DebugTint    = m_debugTint ? 1 : 0;
    auto fa = ctx.objectCB->Allocate(sizeof(fcb));
    if (fa.Cpu) { std::memcpy(fa.Cpu, &fcb, sizeof(fcb)); cmd->SetGraphicsRootConstantBufferView(1, fa.Gpu); }

    for (const Object& o : m_objects) {
        LitObjCB cb;
        XMStoreFloat4x4(&cb.MVP, XMLoadFloat4x4(&o.Model) * camVP);
        cb.Model = o.Model;
        cb.BaseColor = o.Color;
        auto a = ctx.objectCB->Allocate(sizeof(cb));
        if (!a.Cpu) continue;
        std::memcpy(a.Cpu, &cb, sizeof(cb));
        cmd->SetGraphicsRootConstantBufferView(0, a.Gpu);
        m_cube->Draw(cmd);
    }
}

void CsmScene::OnImGui() {
    ImGui::Checkbox("Animate sun", &m_animateLight);
    ImGui::Checkbox("Tint by cascade", &m_debugTint);
    ImGui::SliderFloat("Light elevation", &m_lightElevation, 20.0f, 80.0f, "%.0f deg");
    ImGui::SliderFloat("Light azimuth",   &m_lightAzimuth, 0.0f, 6.28f, "%.2f");
    ImGui::SliderFloat("Split lambda",    &m_lambda, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("Shadow distance", &m_shadowFar, 40.0f, 200.0f, "%.0f");
    ImGui::SliderFloat("Depth bias",      &m_bias, 0.0f, 0.01f, "%.4f");
    ImGui::TextDisabled("%d cascades, %dx%d each", kCascades, kShadowSize, kShadowSize);
}
