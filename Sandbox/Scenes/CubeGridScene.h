#pragma once
#include "Scene/DemoScene.h"
#include "Scene/World.h"

namespace SGE { class Mesh; }

// The original Step-6 demo, now packaged as a switchable scene: an N×N grid of
// cubes, each an ECS entity (Transform + Mesh + Material), spun every frame and
// drawn through RenderSystem. The shared cube Mesh is owned by the app and
// injected at construction.
class CubeGridScene : public SGE::DemoScene {
public:
    explicit CubeGridScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "Cube Grid (ECS)"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

private:
    void Rebuild();

    SGE::Mesh*  m_cube      = nullptr;
    SGE::World  m_world;
    int         m_gridSize  = 3;    // N×N (capped so the per-frame arena fits)
    float       m_spinSpeed = 0.6f;
};
