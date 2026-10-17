#pragma once
#include "Scene/DemoScene.h"
#include "Renderer/DX12/RootSignature.h"
#include "Renderer/DX12/GraphicsPipeline.h"
#include "Renderer/DX12/RenderTexture.h"
#include "Renderer/DX12/SrvHeap.h"
#include "Renderer/RenderGraph/RenderGraph.h"
#include "Renderer/ShaderLibrary.h"

namespace SGE { class Mesh; }

// Screen-space reflections showcase. A geometry pass fills a view-space G-buffer
// (lit colour + reflectivity, normal, position); a resolve pass reflects each
// pixel's view ray, marches it through the depth buffer, and composites the hit
// colour weighted by reflectivity, Fresnel and an edge fade. A reflective floor
// under colored objects shows the classic SSR look — and its limitation (only
// on-screen geometry can be reflected). A/B toggle vs. the raw lit scene.
class SsrScene : public SGE::DemoScene {
public:
    explicit SsrScene(SGE::Mesh* cube, SGE::Mesh* sphere) : m_cube(cube), m_sphere(sphere) {}

    const char* Name()        const override { return "Screen-Space Reflections"; }
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
    SGE::Mesh*            m_sphere = nullptr;
    SGE::ShaderLibrary    m_shaders;
    SGE::RenderGraph      m_graph;
    SGE::SrvHeap          m_srvs;

    SGE::RenderTexture    m_color;   // rgb lit + a reflectivity
    SGE::RenderTexture    m_normal;  // view-space normal
    SGE::RenderTexture    m_posV;    // view-space position

    SGE::RootSignature    m_geoRS, m_resolveRS;
    SGE::GraphicsPipeline m_geoPSO, m_resolvePSO;

    uint32_t m_w = 0, m_h = 0;

    bool  m_enabled   = true;
    int   m_steps     = 64;
    float m_reach     = 22.0f;   // reflection ray length (view units)
    float m_thickness = 0.6f;
    float m_strength  = 1.0f;
    float m_edgeFade  = 0.1f;
    bool  m_ready     = false;
};
