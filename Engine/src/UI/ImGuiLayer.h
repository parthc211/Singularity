#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#include <wrl/client.h>
#include <d3d12.h>
#include <dxgiformat.h>
#include <cstdint>

namespace SGE {

// Thin wrapper around Dear ImGui's Win32 + DX12 backends.
//
// Frame flow (driven by Application):
//   BeginFrame()  -> ImGui::NewFrame      (start building the UI for this frame)
//   ...your ImGui:: widget calls...
//   Render(cmd)   -> ImGui::Render + record draw data into the command list
//
// The DX12 backend needs ONE shader-visible SRV descriptor for its font-atlas
// texture; we own a tiny 1-slot CBV/SRV/UAV heap for exactly that. It must be
// bound on the command list (SetDescriptorHeaps) right before ImGui draws,
// which Render() does for you.
class ImGuiLayer {
public:
    bool Init(HWND hwnd, ID3D12Device* device, uint32_t framesInFlight, DXGI_FORMAT rtvFormat);
    void Shutdown();

    void BeginFrame();
    void Render(ID3D12GraphicsCommandList* cmd);

    // Forward a Win32 message to ImGui so it can drive its own input. Call this
    // from the window message pump. Returns ImGui's handler result (unused by us).
    LRESULT WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);

    // When ImGui owns the mouse/keyboard (cursor over a window, typing in a
    // field), the app should skip its own camera/gameplay input that frame.
    bool WantCaptureMouse()    const;
    bool WantCaptureKeyboard() const;

    bool IsInitialized() const { return m_initialized; }

private:
    // Shader-visible heap holding ImGui's single font-atlas SRV.
    Microsoft::WRL::ComPtr<ID3D12DescriptorHeap> m_srvHeap;
    bool m_initialized = false;
};

} // namespace SGE
