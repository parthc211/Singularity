#include "Scenes/SimdMathScene.h"
#include "Core/Logger.h"
#include "imgui.h"

#include <string>

using namespace SGE;

const char* SimdMathScene::Description() const {
    return "Hand-written SSE/AVX math (Vec4/Mat4/Quat). Correctness is checked "
           "against a scalar reference AND DirectXMath; the benchmark times the "
           "same workload scalar vs SIMD vs DirectXMath. Note: representative "
           "timings need a Release build — Debug leaves both paths un-inlined.";
}

void SimdMathScene::OnLoad(const DemoContext&) {
    m_results  = Math::Run(false); // correctness only — cheap, runs on load
    m_hasBench = false;

    // Log a one-line summary (and any failures) so correctness is verifiable
    // without opening the panel.
    for (const auto& c : m_results.correctness)
        if (!c.passed)
            LogError(std::string("SIMD math MISMATCH: ") + c.name);
    LogInfo(m_results.allCorrect ? "SIMD math: all ops match scalar + DirectXMath."
                                 : "SIMD math: correctness FAILED (see above).");
}

void SimdMathScene::OnImGui() {
    ImGui::Text("Correctness (SIMD vs scalar vs DirectXMath):");
    if (ImGui::BeginTable("correctness", 3,
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Operation");
        ImGui::TableSetupColumn("Result");
        ImGui::TableSetupColumn("Max error");
        ImGui::TableHeadersRow();
        for (const auto& c : m_results.correctness) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(c.name);
            ImGui::TableNextColumn();
            if (c.passed) ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f), "PASS");
            else          ImGui::TextColored(ImVec4(0.95f, 0.4f, 0.4f, 1.0f), "FAIL");
            ImGui::TableNextColumn(); ImGui::Text("%.2e", c.maxError);
        }
        ImGui::EndTable();
    }
    ImGui::Text(m_results.allCorrect ? "All operations match the references."
                                     : "MISMATCH — see failing rows above.");

    ImGui::Separator();
    if (ImGui::Button("Run benchmark")) {
        m_results  = Math::Run(true);
        m_hasBench = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled(m_results.avxUsed ? "(AVX available)" : "(AVX unavailable — SoA row skipped)");

    if (m_hasBench && !m_results.bench.empty()) {
        ImGui::Spacing();
        ImGui::Text("Timing — best of 12 (lower is better):");
        if (ImGui::BeginTable("bench", 5,
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Workload");
            ImGui::TableSetupColumn("Scalar (ms)");
            ImGui::TableSetupColumn("SIMD (ms)");
            ImGui::TableSetupColumn("DirectXMath (ms)");
            ImGui::TableSetupColumn("SIMD speedup");
            ImGui::TableHeadersRow();
            for (const auto& b : m_results.bench) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::TextUnformatted(b.name);
                ImGui::TableNextColumn(); ImGui::Text("%.2f", b.scalarMs);
                ImGui::TableNextColumn();
                if (b.simdMs >= 0.0) ImGui::Text("%.2f", b.simdMs); else ImGui::TextUnformatted("-");
                ImGui::TableNextColumn();
                if (b.dxmathMs >= 0.0) ImGui::Text("%.2f", b.dxmathMs); else ImGui::TextUnformatted("-");
                ImGui::TableNextColumn();
                if (b.simdMs > 0.0) ImGui::Text("%.2fx", b.scalarMs / b.simdMs);
                else                ImGui::TextUnformatted("-");
            }
            ImGui::EndTable();
        }
    }
}
