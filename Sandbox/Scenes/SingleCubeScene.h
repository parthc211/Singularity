#pragma once
#include "Scene/DemoScene.h"
#include "Scene/World.h"
#include "Scene/Entity.h"

namespace SGE { class Mesh; }

// A second scene to exercise the switcher: one cube whose spin, scale, and tint
// are driven live from its ImGui panel. Shows that each scene owns independent
// state (its own World) and its own controls.
class SingleCubeScene : public SGE::DemoScene {
public:
    explicit SingleCubeScene(SGE::Mesh* cube) : m_cube(cube) {}

    const char* Name()        const override { return "Single Cube"; }
    const char* Description() const override;

    void OnLoad(const SGE::DemoContext& ctx) override;
    void OnUnload() override;
    void OnUpdate(const SGE::DemoContext& ctx) override;
    void OnRender(const SGE::DemoContext& ctx) override;
    void OnImGui() override;

private:
    SGE::Mesh*   m_cube = nullptr;
    SGE::World   m_world;
    SGE::Entity  m_entity;
    float        m_spin     = 1.0f;
    float        m_scale    = 1.5f;
    float        m_color[4] = { 0.6f, 0.8f, 1.0f, 1.0f };
};
