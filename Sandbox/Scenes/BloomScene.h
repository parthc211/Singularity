#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RenderTexture.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/ShaderLibrary.h"

#include <DirectXMath.h>
#include <vector>

namespace SGE { class Mesh; }

// HDR + bloom post-processing showcase. Geometry is rendered (emissive colours,
// some > 1) into a floating-point HDR target; a bright-pass extracts the glowing
// parts; a separable Gaussian blur (ping-pong) spreads them; a final pass adds
// the bloom back and tonemaps HDR -> the 8-bit back buffer.
class BloomScene : public SGE::DemoScene {
public:
    explicit BloomScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "HDR + Bloom"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override {
        pos[0] = 0.0f; pos[1] = 4.0f; pos[2] = -14.0f;
        yaw = 0.0f; pitch = -0.18f;
        return true;
    }

private:
    struct Obj { float baseAngle, radius, y, scale, spin; DirectX::XMFLOAT4 color; };

    bool BuildPipelines(const SGE::DemoContext& ctx);
    bool CreateTargets(ID3D12Device* device, uint32_t w, uint32_t h);
    void BuildObjects();
    D3D12_GPU_DESCRIPTOR_HANDLE SlotGpu(uint32_t slot) const;
    D3D12_CPU_DESCRIPTOR_HANDLE SlotCpu(uint32_t slot) const;

    SGE::Mesh*            m_cube = nullptr;
    SGE::RenderTexture    m_sceneHDR, m_bloomA, m_bloomB;
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_srvHeap; // shared: scene=0, bloomA=1, bloomB=2
    uint32_t              m_srvStride = 0;
    uint32_t              m_targetW = 0, m_targetH = 0;

    SGE::ShaderLibrary    m_shaders;
    SGE::RootSignature    m_geoRootSig, m_postRootSig, m_compositeRootSig;
    SGE::GraphicsPipeline m_geoPSO, m_brightPSO, m_blurPSO, m_compositePSO;

    std::vector<Obj>      m_objects;
    float m_time      = 0.0f;
    bool  m_animate   = true;
    float m_threshold = 1.0f;
    float m_intensity = 1.0f;
    float m_exposure  = 1.0f;
    int   m_blurPasses = 3;
    bool  m_ready     = false;
};
