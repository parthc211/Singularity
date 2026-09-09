#pragma once
#include "Scene/DemoScene.h"
#include "Scene/World.h"
#include "Renderer/DX12/RenderTexture.h"
#include "Renderer/RenderGraph/RenderGraph.h"

namespace SGE { class Mesh; }

// Render-graph showcase. Runs a small multi-pass frame THROUGH the RenderGraph
// and makes the graph's work visible: the ImGui panel prints the compiled
// schedule, the barriers the graph inserted before each pass, and which passes
// were culled. A checkbox adds/removes a consumer of an auxiliary target so you
// can watch the graph cull the now-unused pass (and its barriers) live.
class RenderGraphScene : public SGE::DemoScene {
public:
    explicit RenderGraphScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "Render Graph"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

    bool PreferredCamera(float pos[3], float& yaw, float& pitch) const override;

private:
    void Rebuild();

    SGE::Mesh*        m_cube = nullptr;
    SGE::World        m_world;
    SGE::RenderGraph  m_graph;
    SGE::RenderTexture m_aux;   // auxiliary target for the cull demonstration

    int   m_gridSize   = 5;
    float m_spinSpeed  = 0.5f;
    bool  m_consumeAux = true;  // when false, the AuxClear pass has no reader -> culled
    bool  m_ready      = false;
};
