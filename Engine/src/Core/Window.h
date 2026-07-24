#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>

#include <cstdint>
#include <functional>

namespace SGE {

class Window
{
public:
    using ResizeCallback  = std::function<void(uint32_t width, uint32_t height)>;
    using MessageCallback = std::function<void(UINT msg, WPARAM wParam, LPARAM lParam)>;

    bool Initialize(HINSTANCE hInstance, const wchar_t* title, uint32_t width, uint32_t height);
    void Shutdown();

    // Pumps pending Win32 messages. Returns false when WM_QUIT is received.
    bool ProcessMessages();

    void SetResizeCallback(ResizeCallback cb)   { m_resizeCallback  = std::move(cb); }
    // Every Win32 message is forwarded here before internal handling (e.g. for InputSystem).
    void SetMessageCallback(MessageCallback cb)  { m_messageCallback = std::move(cb); }

    HWND     GetHandle() const  { return m_hwnd;   }
    uint32_t GetWidth()  const  { return m_width;  }
    uint32_t GetHeight() const  { return m_height; }

private:
    static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);

    HWND             m_hwnd            = nullptr;
    uint32_t         m_width           = 0;
    uint32_t         m_height          = 0;
    ResizeCallback   m_resizeCallback;
    MessageCallback  m_messageCallback;
};

} // namespace SGE
