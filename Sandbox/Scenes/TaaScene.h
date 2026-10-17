#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/RenderTexture.h"
#include "Renderer/DX12/SrvHeap.h"
#include "Renderer/RenderGraph/RenderGraph.h"
#include "Renderer/ShaderLibrary.h"

#include <DirectXMath.h>

namespace SGE { class Mesh; }

// Temporal anti-aliasing showcase. Geometry is rendered with a sub-pixel jittered
// projection into a colour + motion-vector target; a resolve pass reprojects the
// previous frame's result along the motion vectors and blends (history-heavy),
// clamping history to the local colour neighbourhood to kill ghosting. Over
// frames the jitter accumulates into supersampled edges. An A/B toggle shows the
// raw aliased image for comparison; motion vectors are produced from un-jittered
// clip positions so camera motion reprojects correctly.
class TaaScene : public SGE::DemoScene {
public:
    explicit TaaScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "Temporal AA"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override;

private:
    bool BuildPipelines(const SGE::DemoContext& ctx);
    bool CreateTargets(const SGE::DemoContext& ctx, uint32_t w, uint32_t h);
    void WriteSrvs(const SGE::DemoContext& ctx);

    SGE::Mesh*            m_cube = nullptr;
    SGE::ShaderLibrary    m_shaders;
    SGE::RenderGraph      m_graph;
    SGE::SrvHeap          m_srvs;

    SGE::RenderTexture    m_color;     // RGBA16F
    SGE::RenderTexture    m_velocity;  // RG16F motion vectors
    SGE::RenderTexture    m_historyA;  // RGBA16F resolved-history ping/pong
    SGE::RenderTexture    m_historyB;

    SGE::RootSignature    m_geoRS, m_resolveRS;
    SGE::GraphicsPipeline m_geoPSO, m_resolvePSO;

    uint32_t m_w = 0, m_h = 0;
    uint32_t m_parity     = 0;
    bool     m_firstFrame = true;
    uint32_t m_jitterIdx  = 0;
    DirectX::XMFLOAT4X4 m_prevViewProj;

    bool  m_taa   = true;
    float m_blend = 0.1f;   // current-frame weight
    bool  m_ready = false;
};
