#include "Scenes/IblScene.h"

#include "Renderer/Renderer.h"
#include "Renderer/Mesh.h"
#include "Renderer/DX12/DynamicUploadBuffer.h"
#include "Renderer/DX12/RootSignatureBuilder.h"
#include "Core/Camera.h"

#include "imgui.h"
#include <DirectXMath.h>

using namespace SGE;
using namespace DirectX;

namespace {
// b0 for the skybox pass.
struct SkyCB {
    XMFLOAT4X4 invViewProj;
    XMFLOAT4   camPos;
    float      exposure;
    float      pad[3];
};
// b0 per sphere.
struct ObjCB {
    XMFLOAT4X4 mvp;
    XMFLOAT4X4 model;
    XMFLOAT4   albedo;
    XMFLOAT4   material;   // x roughness, y metallic
};
// b1 shared across the object pass.
struct FrameCB {
    XMFLOAT4 camPos;
    XMFLOAT4 sunDir;
    float    sunIntensity;
    int      useDirect;
    int      diffuseIbl;
    int      specularIbl;
    float    iblIntensity;
    float    maxMip;
    float    exposure;
    int      gamma;
};
} // namespace

const char* IblScene::Description() const {
    return "Image-based lighting. A procedural HDR sky is baked by compute into "
           "an environment cube, then convolved into an irradiance cube (diffuse) "
           "and a prefiltered-mip cube (specular), plus a BRDF LUT — the split-sum "
           "approximation. The sphere grid is lit ENTIRELY from that environment: "
           "roughness varies left-to-right, metalness top-to-bottom. Toggle the "
           "diffuse/specular IBL terms and the analytic sun to see each contribution.";
}

void IblScene::OnLoad(const DemoContext& ctx) {
    m_shaders.Initialize(L"Shaders");
    if (!BuildPipelines(ctx))
        return;
    m_srvs.Create(ctx.device, 4);
    if (!m_ibl.Bake(ctx.device, ctx.renderer->GetCommandQueue(), m_shaders, m_bake))
        return;
    WriteSrvs(ctx);
    m_ready = true;
}

void IblScene::OnUnload() {
    m_ready = false;
    m_ibl.Reset();
    m_srvs.Reset();
    m_skyPSO.Reset();  m_skyRS.Reset();
    m_objPSO.Reset();  m_objRS.Reset();
    m_shaders.Shutdown();
}

bool IblScene::BuildPipelines(const DemoContext& ctx) {
    ID3D12Device* device = ctx.device;

    // --- Skybox: sky CBV (b0) + env cube SRV table (t0) + linear-clamp sampler. ---
    if (!RootSignatureBuilder().Cbv(0).SrvTable(0, 1).SamplerLinearClamp(0)
             .Build(device, m_skyRS))
        return false;
    auto svs = m_shaders.GetOrCompile(L"IblSkybox.hlsl", "VSMain", "vs_6_0");
    auto sps = m_shaders.GetOrCompile(L"IblSkybox.hlsl", "PSMain", "ps_6_0");
    if (!svs || !sps) return false;
    GraphicsPipelineDesc sky;
    sky.rootSignature = m_skyRS.Get();
    sky.vs = svs; sky.ps = sps;
    sky.depthEnable = false;                 // fullscreen background
    sky.cullMode    = D3D12_CULL_MODE_NONE;
    if (!m_skyPSO.Create(device, sky)) return false;

    // --- Objects: per-object CBV (b0) + frame CBV (b1) + IBL SRV table (t0..t2). ---
    if (!RootSignatureBuilder().Cbv(0).Cbv(1).SrvTable(0, 3).SamplerLinearClamp(0)
             .Build(device, m_objRS))
        return false;
    auto ovs = m_shaders.GetOrCompile(L"IblObject.hlsl", "VSMain", "vs_6_0");
    auto ops = m_shaders.GetOrCompile(L"IblObject.hlsl", "PSMain", "ps_6_0");
    if (!ovs || !ops) return false;
    GraphicsPipelineDesc obj;
    obj.rootSignature = m_objRS.Get();
    obj.vs = ovs; obj.ps = ops;
    obj.depthEnable = true;
    obj.dsvFormat   = DXGI_FORMAT_D32_FLOAT;
    return m_objPSO.Create(device, obj);
}

void IblScene::WriteSrvs(const DemoContext& ctx) {
    m_ibl.CreateEnvSrvInto(ctx.device,         m_srvs.Cpu(0));
    m_ibl.CreateIrradianceSrvInto(ctx.device,  m_srvs.Cpu(1));
    m_ibl.CreatePrefilteredSrvInto(ctx.device, m_srvs.Cpu(2));
    m_ibl.CreateBrdfLutSrvInto(ctx.device,     m_srvs.Cpu(3));
}

void IblScene::OnRender(const DemoContext& ctx) {
    if (!m_ready) return;

    ID3D12GraphicsCommandList* cmd = ctx.cmd;
    Renderer* r = ctx.renderer;

    // Re-bake when the sun/exposure changed. Bake is blocking + creates fresh
    // resources, so drain the GPU first (the in-flight frame may still reference
    // the old cubes) and re-point the SRV heap afterwards.
    if (m_rebakeQueued) {
        r->WaitForGPU();
        m_ibl.Bake(ctx.device, r->GetCommandQueue(), m_shaders, m_bake);
        WriteSrvs(ctx);
        m_rebakeQueued = false;
    }

    const XMMATRIX viewProj    = ctx.camera->GetViewProjection();
    const XMMATRIX invViewProj = XMMatrixInverse(nullptr, viewProj);

    m_graph.Begin(cmd);
    m_graph.SetProfiler(&r->GetProfiler());
    const RgHandle back  = m_graph.Import("BackBuffer", r->BackBufferResource(), RgState::RenderTarget);
    const RgHandle depth = m_graph.Import("SceneDepth", r->DepthResource(),      RgState::DepthWrite);

    // ---- Skybox: fullscreen env-cube background (no depth). ----
    m_graph.AddPass("Skybox",
        { Write(back, RgState::RenderTarget) },
        [this, r, invViewProj, &ctx](ID3D12GraphicsCommandList* c) {
            D3D12_CPU_DESCRIPTOR_HANDLE rtv = r->GetBackBufferRTV();
            c->OMSetRenderTargets(1, &rtv, FALSE, nullptr);
            c->SetGraphicsRootSignature(m_skyRS.Get());
            c->SetPipelineState(m_skyPSO.Get());
            SkyCB sky = {};
            XMStoreFloat4x4(&sky.invViewProj, invViewProj);
            sky.camPos   = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2], 1.0f };
            sky.exposure = m_bake.exposure;
            ctx.objectCB->BindCbv(c, 0, sky);
            m_srvs.BindTable(c, 1, 0);            // env cube at slot 0
            c->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
            c->IASetVertexBuffers(0, 0, nullptr);
            c->DrawInstanced(3, 1, 0, 0);
        },
        /*sideEffect*/ true);

    // ---- Objects: the sphere grid, lit from the environment. ----
    m_graph.AddPass("Objects",
        { Write(back, RgState::RenderTarget), Write(depth, RgState::DepthWrite) },
        [this, r, viewProj, &ctx](ID3D12GraphicsCommandList* c) {
            r->BindBackBufferTargets(c);          // back buffer + depth + viewport
            c->SetGraphicsRootSignature(m_objRS.Get());
            c->SetPipelineState(m_objPSO.Get());
            c->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

            FrameCB fr = {};
            fr.camPos       = { ctx.cameraPos[0], ctx.cameraPos[1], ctx.cameraPos[2], 1.0f };
            fr.sunDir       = { m_bake.sunDir[0], m_bake.sunDir[1], m_bake.sunDir[2], 0.0f };
            fr.sunIntensity = m_useDirect ? 3.0f : 0.0f;
            fr.useDirect    = m_useDirect ? 1 : 0;
            fr.diffuseIbl   = m_diffuseIbl ? 1 : 0;
            fr.specularIbl  = m_specularIbl ? 1 : 0;
            fr.iblIntensity = m_iblIntensity;
            fr.maxMip       = m_ibl.PrefilteredMaxMip();
            fr.exposure     = m_bake.exposure;
            fr.gamma        = 1;
            ctx.objectCB->BindCbv(c, 1, fr);
            m_srvs.BindTable(c, 2, 1);            // irradiance/prefiltered/brdfLut at slots 1..3

            const int   n       = m_gridN;
            const float spacing = 2.4f;
            const float offset  = (n - 1) * 0.5f * spacing;
            for (int gy = 0; gy < n; ++gy) {
                for (int gx = 0; gx < n; ++gx) {
                    const XMMATRIX model =
                        XMMatrixScaling(0.9f, 0.9f, 0.9f) *
                        XMMatrixTranslation(gx * spacing - offset,
                                            gy * spacing - offset, 0.0f);
                    ObjCB o = {};
                    XMStoreFloat4x4(&o.mvp,   model * viewProj);
                    XMStoreFloat4x4(&o.model, model);
                    o.albedo   = { m_albedo[0], m_albedo[1], m_albedo[2], 1.0f };
                    o.material = { (n > 1) ? float(gx) / float(n - 1) : 0.5f,   // roughness ->
                                   (n > 1) ? float(gy) / float(n - 1) : 1.0f,   // metalness v
                                   0.0f, 0.0f };
                    if (ctx.objectCB->BindCbv(c, 0, o))
                        m_sphere->Draw(c);
                }
            }
        },
        /*sideEffect*/ true);

    m_graph.Execute();
}

void IblScene::OnImGui() {
    ImGui::SliderInt("Grid size", &m_gridN, 2, 8);
    ImGui::TextDisabled("roughness -> across | metalness v down");
    ImGui::Separator();
    ImGui::Checkbox("Diffuse IBL",  &m_diffuseIbl);
    ImGui::Checkbox("Specular IBL", &m_specularIbl);
    ImGui::Checkbox("Analytic sun", &m_useDirect);
    ImGui::SliderFloat("IBL intensity", &m_iblIntensity, 0.0f, 3.0f, "%.2f");
    ImGui::ColorEdit3("Albedo", m_albedo);
    ImGui::Separator();
    ImGui::TextDisabled("Environment (re-bakes on change)");
    bool rebake = false;
    rebake |= ImGui::SliderFloat3("Sun dir", m_bake.sunDir, -1.0f, 1.0f, "%.2f");
    rebake |= ImGui::SliderFloat("Sun intensity", &m_bake.sunIntensity, 0.0f, 120.0f, "%.0f");
    rebake |= ImGui::SliderFloat("Sky exposure", &m_bake.exposure, 0.1f, 3.0f, "%.2f");
    if (rebake) m_rebakeQueued = true;
    ImGui::TextDisabled("Baked: env 256^3 + irradiance + 5 prefilter mips + BRDF LUT");
}

bool IblScene::PreferredCamera(float pos[3], float& yaw, float& pitch) const {
    pos[0] = 0.0f; pos[1] = 0.0f; pos[2] = -16.0f;
    yaw = 0.0f; pitch = 0.0f;
    return true;
}
