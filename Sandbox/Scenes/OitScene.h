#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/RenderTexture.h"
#include "Renderer/DX12/SrvHeap.h"
#include "Renderer/RenderGraph/RenderGraph.h"
#include "Renderer/ShaderLibrary.h"

namespace SGE { class Mesh; }

// Order-independent transparency (weighted-blended OIT). Opaque geometry is drawn
// first; transparent geometry is then accumulated into an additive colour target
// and a multiplicative revealage target with depth-weighted blending, so the
// surfaces composite to the same result in ANY draw order. A composite pass
// resolves them over the opaque image. An A/B toggle switches to naive
// order-dependent alpha blending so the overlap artifacts are visible for
// comparison (orbit the camera and watch the colours flip in naive mode).
class OitScene : public SGE::DemoScene {
public:
    OitScene(SGE::Mesh* cube, SGE::Mesh* sphere) : m_cube(cube), m_sphere(sphere) {}

    const char* Name()        const override { return "Order-Independent Transparency"; }
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

    SGE::Mesh*            m_cube = nullptr;
    SGE::Mesh*            m_sphere = nullptr;
    SGE::ShaderLibrary    m_shaders;
    SGE::RenderGraph      m_graph;
    SGE::SrvHeap          m_srvs;

    SGE::RenderTexture    m_accum;    // RGBA16F additive
    SGE::RenderTexture    m_reveal;   // R16F multiplicative (1-alpha)

    SGE::RootSignature    m_objRS, m_compositeRS;
    SGE::GraphicsPipeline m_opaquePSO, m_accumPSO, m_naivePSO, m_compositePSO;

    uint32_t m_w = 0, m_h = 0;
    bool  m_wboit  = true;
    float m_alpha  = 0.5f;
    bool  m_spin   = true;
    float m_time   = 0.0f;
    bool  m_ready  = false;
};
