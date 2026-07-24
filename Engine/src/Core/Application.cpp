#include "Core/Application.h"
#include "Core/Logger.h"

namespace SGE {

bool Application::Initialize(HINSTANCE hInstance, const wchar_t* title, uint32_t width, uint32_t height)
{
    if (!m_window.Initialize(hInstance, title, width, height))
        return false;

    if (!m_renderer.Initialize(m_window.GetHandle(), width, height))
        return false;

    // Wire up resize so the swap chain tracks window size changes.
    m_window.SetResizeCallback([this](uint32_t w, uint32_t h) {
        m_renderer.OnResize(w, h);
        OnResize(w, h);
    });

    // ImGui must exist before any window message is routed to it. Its Win32
    // backend is created with our HWND; the DX12 backend with our device and
    // the swap chain's RTV format (R8G8B8A8_UNORM).
    if (!m_imgui.Init(m_window.GetHandle(), m_renderer.GetDevice(),
                      FrameCount, DXGI_FORMAT_R8G8B8A8_UNORM))
        return false;

    // Register raw mouse input and route all Win32 messages to ImGui first
    // (so it can update its own input state) and then to InputSystem.
    m_input.Initialize(m_window.GetHandle());
    m_window.SetMessageCallback([this](UINT msg, WPARAM wParam, LPARAM lParam) {
        m_imgui.WndProc(m_window.GetHandle(), msg, wParam, lParam);
        m_input.OnMessage(msg, wParam, lParam);
    });

    OnStartup();

    m_lastFrameTime = std::chrono::high_resolution_clock::now();
    LogInfo("Application initialized.");
    return true;
}

void Application::Run()
{
    LogInfo("Entering main loop.");

    while (true)
    {
        // Snapshot previous key state before new messages arrive so
        // IsKeyJustPressed / IsKeyJustReleased see a clean prev-vs-current diff.
        m_input.BeginFrame();

        if (!m_window.ProcessMessages())
            break;

        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - m_lastFrameTime).count();
        m_lastFrameTime = now;
        m_deltaTime = dt < 0.05f ? dt : 0.05f;

        m_renderer.BeginFrame();
        m_imgui.BeginFrame();   // open a new ImGui frame before any widget calls

        OnRender();             // scene draws into the bound back buffer
        OnImGui();              // build this frame's ImGui windows

        // Record ImGui's draw data last, so the UI composites on top of the
        // scene, then close out the GPU frame.
        m_imgui.Render(m_renderer.GetCommandList());
        m_renderer.EndFrame();
        m_input.EndFrame();
    }

    LogInfo("Exiting main loop.");
}

void Application::Shutdown()
{
    OnShutdown();
    // Drain the GPU before releasing ImGui's GPU resources (font texture, PSO),
    // then tear ImGui down before the device goes away (constraint 6).
    m_renderer.WaitForGPU();
    m_imgui.Shutdown();
    m_renderer.Shutdown();
    m_window.Shutdown();
    LogInfo("Application shut down.");
}

} // namespace SGE
