#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/ComputePipeline.h"
#include "Renderer/DX12/RenderTexture.h"
#include "Renderer/DX12/SrvHeap.h"
#include "Renderer/RenderGraph/RenderGraph.h"
#include "Renderer/ShaderLibrary.h"

namespace SGE { class Mesh; }

// Post-processing stack showcase. A grid of lit + emissive spheres is rendered
// to an HDR target, then a composable chain of fullscreen/compute passes runs
// over it through the render graph: eye-adaptation (compute) -> bloom
// (bright-pass + separable blur) -> composite (ACES tonemap + colour grade +
// vignette + chromatic aberration + film grain). Every effect is a toggle, so
// the ImGui panel builds the stack a node at a time.
class PostProcessScene : public SGE::DemoScene {
public:
    explicit PostProcessScene(SGE::Mesh* sphere) : m_sphere(sphere) {}

    const char* Name()        const override { return "Post-Processing"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override;

private:
    bool BuildPipelines(const SGE::DemoContext& ctx);
    bool CreateTargets(const SGE::DemoContext& ctx, uint32_t w, uint32_t h);
    void WriteSrvs(const SGE::DemoContext& ctx);

    SGE::Mesh*            m_sphere = nullptr;
    SGE::ShaderLibrary    m_shaders;
    SGE::RenderGraph      m_graph;
    SGE::SrvHeap          m_srvs;

    SGE::RenderTexture    m_hdr;      // scene render target (RGBA16F)
    SGE::RenderTexture    m_bloomA;   // half-res bloom ping/pong
    SGE::RenderTexture    m_bloomB;
    ComPtr<ID3D12Resource> m_adapted; // 1x1 R32F adapted luminance (persists)

    SGE::RootSignature    m_geoRS, m_brightRS, m_blurRS, m_exposureRS, m_compositeRS;
    SGE::GraphicsPipeline m_geoPSO, m_brightPSO, m_blurPSO, m_compositePSO;
    SGE::ComputePipeline  m_exposurePSO;

    uint32_t m_w = 0, m_h = 0, m_bw = 0, m_bh = 0;

    // ImGui-controlled composable parameters.
    bool  m_bloom        = true;
    float m_bloomStrength= 0.6f;
    float m_threshold    = 1.0f;
    int   m_blurIters    = 4;
    bool  m_autoExposure = true;
    float m_exposure     = 1.0f;
    float m_keyValue     = 0.18f;
    float m_adaptSpeed   = 1.5f;
    float m_vignette     = 0.5f;
    float m_ca           = 0.0f;
    float m_grain        = 0.04f;
    float m_contrast     = 1.05f;
    float m_saturation   = 1.05f;
    float m_temperature  = 0.0f;
    float m_time         = 0.0f;
    bool  m_ready        = false;
};
