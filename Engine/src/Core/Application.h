#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>

#include "Core/Window.h"
#include "Core/InputSystem.h"
#include "Renderer/Renderer.h"
#include "UI/ImGuiLayer.h"

#include <chrono>
#include <cstdint>

namespace SGE {

class Application
{
public:
    bool Initialize(HINSTANCE hInstance, const wchar_t* title, uint32_t width, uint32_t height);
    void Run();
    void Shutdown();

protected:
    virtual void OnStartup()                             {}
    virtual void OnRender()                              {}
    // ImGui widget calls go here. Runs each frame between the renderer's scene
    // pass and the ImGui draw submission (inside ImGui's NewFrame/Render bracket).
    virtual void OnImGui()                               {}
    virtual void OnShutdown()                            {}
    virtual void OnResize(uint32_t /*w*/, uint32_t /*h*/) {}

    Renderer&    GetRenderer() { return m_renderer; }
    Window&      GetWindow()   { return m_window;   }
    InputSystem& GetInput()    { return m_input;    }
    ImGuiLayer&  GetImGui()    { return m_imgui;    }

    // True when ImGui currently owns the mouse/keyboard — gate camera/gameplay
    // input on these so dragging a UI panel doesn't also fly the camera.
    bool IsUICapturingMouse()    const { return m_imgui.WantCaptureMouse();    }
    bool IsUICapturingKeyboard() const { return m_imgui.WantCaptureKeyboard(); }

    // Seconds elapsed since the previous frame — capped at 50 ms to avoid large jumps.
    float GetDeltaTime() const { return m_deltaTime; }

private:
    Window      m_window;
    Renderer    m_renderer;
    InputSystem m_input;
    ImGuiLayer  m_imgui;

    float m_deltaTime = 0.0f;
    std::chrono::high_resolution_clock::time_point m_lastFrameTime;
};

} // namespace SGE
