#include "UI/ImGuiLayer.h"
#include "Core/Logger.h"

#include "imgui.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_dx12.h"

// Declared in imgui_impl_win32.cpp; lets us hand raw Win32 messages to ImGui.
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

namespace SGE {

bool ImGuiLayer::Init(HWND hwnd, ID3D12Device* device, uint32_t framesInFlight, DXGI_FORMAT rtvFormat) {
    // The DX12 backend samples its font atlas through one SRV, so we need a
    // SHADER_VISIBLE descriptor heap (the GPU can only read descriptors from
    // shader-visible heaps). One slot is enough for the basic single-font setup.
    D3D12_DESCRIPTOR_HEAP_DESC heapDesc = {};
    heapDesc.Type           = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    heapDesc.NumDescriptors = 1;
    heapDesc.Flags          = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    if (FAILED(device->CreateDescriptorHeap(&heapDesc, IID_PPV_ARGS(&m_srvHeap)))) {
        LogError("ImGuiLayer: failed to create SRV descriptor heap");
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    if (!ImGui_ImplWin32_Init(hwnd)) {
        LogError("ImGuiLayer: ImGui_ImplWin32_Init failed");
        return false;
    }
    if (!ImGui_ImplDX12_Init(
            device,
            static_cast<int>(framesInFlight),
            rtvFormat,
            m_srvHeap.Get(),
            m_srvHeap->GetCPUDescriptorHandleForHeapStart(),
            m_srvHeap->GetGPUDescriptorHandleForHeapStart())) {
        LogError("ImGuiLayer: ImGui_ImplDX12_Init failed");
        return false;
    }

    m_initialized = true;
    LogInfo("ImGui initialized.");
    return true;
}

void ImGuiLayer::Shutdown() {
    if (!m_initialized) return;
    // Backends release their GPU resources (font texture, PSO); the caller must
    // have drained the GPU first (constraint 6).
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    m_srvHeap.Reset();
    m_initialized = false;
}

void ImGuiLayer::BeginFrame() {
    if (!m_initialized) return;
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
}

void ImGuiLayer::Render(ID3D12GraphicsCommandList* cmd) {
    if (!m_initialized) return;
    ImGui::Render();
    // ImGui draws into whatever render target is currently bound (the back
    // buffer, set in Renderer::BeginFrame). It manages its own PSO/root sig, but
    // we must bind the font-atlas heap first so its pixel shader can sample it.
    ID3D12DescriptorHeap* heaps[] = { m_srvHeap.Get() };
    cmd->SetDescriptorHeaps(1, heaps);
    ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), cmd);
}

LRESULT ImGuiLayer::WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (!m_initialized) return 0;
    return ImGui_ImplWin32_WndProcHandler(hwnd, msg, wParam, lParam);
}

bool ImGuiLayer::WantCaptureMouse()    const { return m_initialized && ImGui::GetIO().WantCaptureMouse;    }
bool ImGuiLayer::WantCaptureKeyboard() const { return m_initialized && ImGui::GetIO().WantCaptureKeyboard; }

} // namespace SGE
