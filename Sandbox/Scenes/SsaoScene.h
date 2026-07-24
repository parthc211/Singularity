#pragma once
#include "Scene/DemoScene.h"
#include "Scene/World.h"
#include "Renderer/DX12/GBuffer.h"
#include "Renderer/DX12/RenderTexture.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/ShaderLibrary.h"

#include <DirectXMath.h>
#include <vector>

namespace SGE { class Mesh; }

// Screen-space ambient occlusion, built on the deferred G-buffer (reusing the
// GeometryPass + RenderSystem from Step 9). Geometry -> G-buffer (albedo/normal/
// position); an SSAO pass estimates per-pixel occlusion from hemisphere samples;
// a blur removes the noise; a composite modulates albedo by AO. A debug mode
// shows raw AO / albedo-only so the contribution is visible.
class SsaoScene : public SGE::DemoScene {
public:
    explicit SsaoScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "SSAO"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override {
        pos[0] = 0.0f; pos[1] = 8.0f; pos[2] = -15.0f;
        yaw = 0.0f; pitch = -0.35f;
        return true;
    }

private:
    static constexpr int kKernelSize = 32;

    bool BuildPipelines(const SGE::DemoContext& ctx);
    bool CreateTargets(ID3D12Device* device, uint32_t w, uint32_t h);
    void BuildScene();
    void BuildKernel();
    D3D12_GPU_DESCRIPTOR_HANDLE SlotGpu(uint32_t slot) const;
    D3D12_CPU_DESCRIPTOR_HANDLE SlotCpu(uint32_t slot) const;

    SGE::Mesh*            m_cube = nullptr;
    SGE::World            m_world;
    SGE::GBuffer          m_gbuffer;
    SGE::RenderTexture    m_ao, m_aoBlur;        // R8 occlusion + blurred
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_srvHeap; // 0=N,1=Pos,2=AO,3=Albedo,4=AOblur
    uint32_t              m_srvStride = 0;
    uint32_t              m_targetW = 0, m_targetH = 0;

    SGE::ShaderLibrary    m_shaders;
    SGE::RootSignature    m_geoRootSig, m_post1RootSig, m_post2RootSig;
    SGE::GraphicsPipeline m_geoPSO, m_ssaoPSO, m_blurPSO, m_compositePSO;

    std::vector<DirectX::XMFLOAT4> m_kernel;

    float m_radius   = 1.4f;
    float m_bias     = 0.03f;
    float m_power    = 2.0f;
    float m_strength = 1.6f;
    float m_ambient  = 0.15f;
    int   m_samples  = 24;
    int   m_mode     = 0; // 0 albedo*AO, 1 AO only, 2 albedo only
    bool  m_ready    = false;
};
