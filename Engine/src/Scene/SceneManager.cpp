#include "Scene/SceneManager.h"
#include "Renderer/Renderer.h" // WaitForGPU before releasing a scene's GPU resources
#include "imgui.h"

namespace SGE {

void SceneManager::Add(std::unique_ptr<DemoScene> scene) {
    m_scenes.push_back(std::move(scene));
}

void SceneManager::SetActiveIndex(int index, const DemoContext& ctx) {
    if (index < 0 || index >= static_cast<int>(m_scenes.size()) || index == m_active)
        return;
    if (m_active >= 0)
        m_scenes[m_active]->OnUnload();
    m_active = index;
    m_scenes[m_active]->OnLoad(ctx);
}

void SceneManager::Update(const DemoContext& ctx) {
    // Apply a deferred switch at the very start of the frame, before any draws
    // this frame reference the outgoing scene's resources. Drain the GPU first so
    // OnUnload can safely release resources earlier frames may still be using.
    if (m_pending >= 0 && m_pending < static_cast<int>(m_scenes.size()) && m_pending != m_active) {
        if (ctx.renderer)
            ctx.renderer->WaitForGPU();
        if (m_active >= 0)
            m_scenes[m_active]->OnUnload();
        m_active = m_pending;
        m_scenes[m_active]->OnLoad(ctx);
    }
    m_pending = -1;

    if (m_active >= 0)
        m_scenes[m_active]->OnUpdate(ctx);
}

void SceneManager::Render(const DemoContext& ctx) {
    if (m_active >= 0)
        m_scenes[m_active]->OnRender(ctx);
}

void SceneManager::OnImGui(const DemoContext& ctx) {
    ImGui::SetNextWindowPos({ 10, 230 }, ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({ 340, 0 }, ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Scene")) {
        const char* current = (m_active >= 0) ? m_scenes[m_active]->Name() : "(none)";
        if (ImGui::BeginCombo("Demo", current)) {
            for (int i = 0; i < static_cast<int>(m_scenes.size()); ++i) {
                const bool selected = (i == m_active);
                if (ImGui::Selectable(m_scenes[i]->Name(), selected))
                    RequestSwitch(i); // applied next frame (see Update)
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        if (m_active >= 0) {
            const char* desc = m_scenes[m_active]->Description();
            if (desc && desc[0]) {
                ImGui::Separator();
                ImGui::TextWrapped("%s", desc);
            }
            ImGui::Separator();
            m_scenes[m_active]->OnImGui();
        }
    }
    ImGui::End();
}

DemoScene* SceneManager::Active() const {
    return (m_active >= 0) ? m_scenes[m_active].get() : nullptr;
}

} // namespace SGE
