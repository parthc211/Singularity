#include "Core/Window.h"
#include "Core/Logger.h"

namespace SGE {

bool Window::Initialize(HINSTANCE hInstance, const wchar_t* title, uint32_t width, uint32_t height)
{
    m_width  = width;
    m_height = height;

    WNDCLASSEXW wc = {};
    wc.cbSize        = sizeof(wc);
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc   = WndProc;
    wc.hInstance     = hInstance;
    wc.hCursor       = LoadCursorW(nullptr, IDC_ARROW);
    wc.lpszClassName = L"SingularityWindow";

    if (!RegisterClassExW(&wc)) {
        LogError("Failed to register window class.");
        return false;
    }

    // Calculate window rect so the CLIENT area matches the requested dimensions.
    RECT rect = { 0, 0, static_cast<LONG>(width), static_cast<LONG>(height) };
    AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);

    m_hwnd = CreateWindowExW(
        0,
        L"SingularityWindow",
        title,
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rect.right - rect.left,
        rect.bottom - rect.top,
        nullptr, nullptr,
        hInstance,
        this    // passed to WM_CREATE as lpCreateParams so WndProc can store the pointer
    );

    if (!m_hwnd) {
        LogError("Failed to create window.");
        return false;
    }

    ShowWindow(m_hwnd, SW_SHOWDEFAULT);
    UpdateWindow(m_hwnd);

    LogInfo("Window created.");
    return true;
}

void Window::Shutdown()
{
    if (m_hwnd) {
        DestroyWindow(m_hwnd);
        m_hwnd = nullptr;
    }
    UnregisterClassW(L"SingularityWindow", GetModuleHandleW(nullptr));
}

bool Window::ProcessMessages()
{
    MSG msg = {};
    while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT)
            return false;
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return true;
}

LRESULT CALLBACK Window::WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    Window* self = reinterpret_cast<Window*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));

    // Forward every message to the input system (or whatever listener is registered).
    // self is null during WM_CREATE so the check guards that early case.
    if (self && self->m_messageCallback)
        self->m_messageCallback(msg, wParam, lParam);

    switch (msg) {
    case WM_CREATE: {
        auto* cs = reinterpret_cast<CREATESTRUCTW*>(lParam);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
        return 0;
    }
    case WM_SIZE:
        if (self) {
            self->m_width  = LOWORD(lParam);
            self->m_height = HIWORD(lParam);
            if (self->m_resizeCallback && wParam != SIZE_MINIMIZED)
                self->m_resizeCallback(self->m_width, self->m_height);
        }
        return 0;

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }

    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

} // namespace SGE
