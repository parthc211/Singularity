#include "Core/InputSystem.h"

namespace SGE {

void InputSystem::Initialize(HWND hwnd)
{
    RAWINPUTDEVICE rid = {};
    rid.usUsagePage = 0x01; // HID_USAGE_PAGE_GENERIC
    rid.usUsage     = 0x02; // HID_USAGE_GENERIC_MOUSE
    rid.dwFlags     = 0;
    rid.hwndTarget  = hwnd;
    RegisterRawInputDevices(&rid, 1, sizeof(rid));
}

void InputSystem::BeginFrame()
{
    m_keyPrev = m_keyCurrent;
}

void InputSystem::EndFrame()
{
    m_rawDx      = 0;
    m_rawDy      = 0;
    m_wheelDelta = 0.0f;
    PollControllers();
}

void InputSystem::OnMessage(UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg)
    {
    case WM_KEYDOWN:
    case WM_SYSKEYDOWN:
        if (wParam < 256) m_keyCurrent.set(wParam);
        break;

    case WM_KEYUP:
    case WM_SYSKEYUP:
        if (wParam < 256) m_keyCurrent.reset(wParam);
        break;

    case WM_LBUTTONDOWN: m_mouseButtons[0] = true;  break;
    case WM_LBUTTONUP:   m_mouseButtons[0] = false; break;
    case WM_RBUTTONDOWN: m_mouseButtons[1] = true;  break;
    case WM_RBUTTONUP:   m_mouseButtons[1] = false; break;
    case WM_MBUTTONDOWN: m_mouseButtons[2] = true;  break;
    case WM_MBUTTONUP:   m_mouseButtons[2] = false; break;

    case WM_MOUSEMOVE:
        m_mousePos.x = float(LOWORD(lParam));
        m_mousePos.y = float(HIWORD(lParam));
        break;

    case WM_MOUSEWHEEL:
        m_wheelDelta += float(GET_WHEEL_DELTA_WPARAM(wParam)) / float(WHEEL_DELTA);
        break;

    case WM_INPUT: {
        UINT dataSize = 0;
        GetRawInputData(reinterpret_cast<HRAWINPUT>(lParam), RID_INPUT,
                        nullptr, &dataSize, sizeof(RAWINPUTHEADER));
        if (dataSize == 0 || dataSize > sizeof(RAWINPUT))
            break;

        RAWINPUT ri = {};
        if (GetRawInputData(reinterpret_cast<HRAWINPUT>(lParam), RID_INPUT,
                            &ri, &dataSize, sizeof(RAWINPUTHEADER)) == UINT(-1))
            break;

        // MOUSE_MOVE_ABSOLUTE = 1; relative movement has that bit clear.
        if (ri.header.dwType == RIM_TYPEMOUSE &&
            !(ri.data.mouse.usFlags & MOUSE_MOVE_ABSOLUTE))
        {
            m_rawDx += ri.data.mouse.lLastX;
            m_rawDy += ri.data.mouse.lLastY;
        }
        break;
    }
    }
}

bool InputSystem::IsKeyDown(Key key) const
{
    size_t idx = static_cast<size_t>(key);
    return idx < 256 && m_keyCurrent[idx];
}

bool InputSystem::IsKeyJustPressed(Key key) const
{
    size_t idx = static_cast<size_t>(key);
    return idx < 256 && m_keyCurrent[idx] && !m_keyPrev[idx];
}

bool InputSystem::IsKeyJustReleased(Key key) const
{
    size_t idx = static_cast<size_t>(key);
    return idx < 256 && !m_keyCurrent[idx] && m_keyPrev[idx];
}

bool InputSystem::IsMouseButtonDown(MouseButton btn) const
{
    size_t idx = static_cast<size_t>(btn);
    return idx < 3 && m_mouseButtons[idx];
}

void InputSystem::LockCursor(HWND hwnd)
{
    if (m_cursorLocked) return;
    ShowCursor(FALSE);
    RECT rect;
    GetClientRect(hwnd, &rect);
    MapWindowPoints(hwnd, nullptr, reinterpret_cast<POINT*>(&rect), 2);
    ClipCursor(&rect);
    m_cursorLocked = true;
}

void InputSystem::UnlockCursor()
{
    if (!m_cursorLocked) return;
    ClipCursor(nullptr);
    ShowCursor(TRUE);
    m_cursorLocked = false;
}

void InputSystem::PollControllers()
{
    for (uint32_t i = 0; i < k_maxControllers; ++i)
    {
        DWORD result = XInputGetState(i, &m_ctrlState[i]);
        m_ctrlConnected[i] = (result == ERROR_SUCCESS);
    }
}

float InputSystem::NormalizeStick(int16_t raw, int16_t deadzone)
{
    if (raw > -deadzone && raw < deadzone) return 0.0f;
    float val = float(raw) / 32767.0f;
    if (val < -1.0f) return -1.0f;
    if (val >  1.0f) return  1.0f;
    return val;
}

float InputSystem::NormalizeTrigger(uint8_t raw)
{
    if (raw < XINPUT_GAMEPAD_TRIGGER_THRESHOLD) return 0.0f;
    return float(raw) / 255.0f;
}

bool InputSystem::IsControllerConnected(uint32_t idx) const
{
    return idx < k_maxControllers && m_ctrlConnected[idx];
}

bool InputSystem::IsButtonDown(ControllerButton btn, uint32_t idx) const
{
    if (!IsControllerConnected(idx)) return false;
    return (m_ctrlState[idx].Gamepad.wButtons & static_cast<WORD>(btn)) != 0;
}

DirectX::XMFLOAT2 InputSystem::GetLeftStick(uint32_t idx) const
{
    if (!IsControllerConnected(idx)) return {};
    const auto& g = m_ctrlState[idx].Gamepad;
    return {
        NormalizeStick(g.sThumbLX, XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE),
        NormalizeStick(g.sThumbLY, XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE)
    };
}

DirectX::XMFLOAT2 InputSystem::GetRightStick(uint32_t idx) const
{
    if (!IsControllerConnected(idx)) return {};
    const auto& g = m_ctrlState[idx].Gamepad;
    return {
        NormalizeStick(g.sThumbRX, XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE),
        NormalizeStick(g.sThumbRY, XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE)
    };
}

float InputSystem::GetLeftTrigger(uint32_t idx) const
{
    if (!IsControllerConnected(idx)) return 0.0f;
    return NormalizeTrigger(m_ctrlState[idx].Gamepad.bLeftTrigger);
}

float InputSystem::GetRightTrigger(uint32_t idx) const
{
    if (!IsControllerConnected(idx)) return 0.0f;
    return NormalizeTrigger(m_ctrlState[idx].Gamepad.bRightTrigger);
}

} // namespace SGE
