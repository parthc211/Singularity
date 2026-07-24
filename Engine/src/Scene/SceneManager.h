#pragma once
#include "Scene/DemoScene.h"

#include <memory>
#include <vector>

namespace SGE {

// Owns the set of demo scenes and tracks which one is active. Switching unloads
// the current scene and loads the new one. Drives the active scene's per-frame
// hooks and renders the ImGui switcher dropdown.
class SceneManager {
public:
    void Add(std::unique_ptr<DemoScene> scene);

    // Immediate switch (used once at startup): OnUnload current, OnLoad new.
    void SetActiveIndex(int index, const DemoContext& ctx);

    // Request a switch applied at the start of the next Update. Deferring avoids
    // releasing the outgoing scene's GPU resources mid-frame, while the current
    // command list still references them.
    void RequestSwitch(int index) { m_pending = index; }

    // Applies any pending switch (draining the GPU first, then OnUnload/OnLoad),
    // then drives the active scene's OnUpdate.
    void Update(const DemoContext& ctx);
    void Render(const DemoContext& ctx);

    // Draws a "Scene" window: a dropdown to switch + the active scene's
    // description and its own controls (OnImGui).
    void OnImGui(const DemoContext& ctx);

    DemoScene* Active() const;
    int        ActiveIndex() const { return m_active; }

private:
    std::vector<std::unique_ptr<DemoScene>> m_scenes;
    int m_active  = -1;
    int m_pending = -1; // requested scene, applied at next Update
};

} // namespace SGE
