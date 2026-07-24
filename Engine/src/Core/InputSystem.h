#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#include <Xinput.h>
#include <DirectXMath.h>

#include <bitset>
#include <cstdint>

namespace SGE {

// Virtual-key-backed enum — values match Win32 VK_* codes directly.
enum class Key : uint32_t
{
    A = 'A', B = 'B', C = 'C', D = 'D', E = 'E', F = 'F', G = 'G',
    H = 'H', I = 'I', J = 'J', K = 'K', L = 'L', M = 'M', N = 'N',
    O = 'O', P = 'P', Q = 'Q', R = 'R', S = 'S', T = 'T', U = 'U',
    V = 'V', W = 'W', X = 'X', Y = 'Y', Z = 'Z',

    Num0 = '0', Num1 = '1', Num2 = '2', Num3 = '3', Num4 = '4',
    Num5 = '5', Num6 = '6', Num7 = '7', Num8 = '8', Num9 = '9',

    Space     = VK_SPACE,
    Enter     = VK_RETURN,
    Escape    = VK_ESCAPE,
    Tab       = VK_TAB,
    Backspace = VK_BACK,
    Shift     = VK_SHIFT,
    Ctrl      = VK_CONTROL,
    Alt       = VK_MENU,

    Left  = VK_LEFT,
    Right = VK_RIGHT,
    Up    = VK_UP,
    Down  = VK_DOWN,
    Home  = VK_HOME,

    F1  = VK_F1,  F2  = VK_F2,  F3  = VK_F3,  F4  = VK_F4,
    F5  = VK_F5,  F6  = VK_F6,  F7  = VK_F7,  F8  = VK_F8,
    F9  = VK_F9,  F10 = VK_F10, F11 = VK_F11, F12 = VK_F12,
};

enum class MouseButton : uint32_t { Left = 0, Right = 1, Middle = 2 };

// Values match XINPUT_GAMEPAD_* bit masks so they can be tested directly.
enum class ControllerButton : uint32_t
{
    DPadUp        = XINPUT_GAMEPAD_DPAD_UP,
    DPadDown      = XINPUT_GAMEPAD_DPAD_DOWN,
    DPadLeft      = XINPUT_GAMEPAD_DPAD_LEFT,
    DPadRight     = XINPUT_GAMEPAD_DPAD_RIGHT,
    Start         = XINPUT_GAMEPAD_START,
    Back          = XINPUT_GAMEPAD_BACK,
    LeftThumb     = XINPUT_GAMEPAD_LEFT_THUMB,
    RightThumb    = XINPUT_GAMEPAD_RIGHT_THUMB,
    LeftShoulder  = XINPUT_GAMEPAD_LEFT_SHOULDER,
    RightShoulder = XINPUT_GAMEPAD_RIGHT_SHOULDER,
    A             = XINPUT_GAMEPAD_A,
    B             = XINPUT_GAMEPAD_B,
    X             = XINPUT_GAMEPAD_X,
    Y             = XINPUT_GAMEPAD_Y,
};

class InputSystem
{
public:
    // Registers raw mouse input for the given window — call once after window creation.
    void Initialize(HWND hwnd);

    // Called by Application before OnRender: snapshots previous key state.
    void BeginFrame();
    // Called by Application after OnRender: clears per-frame accumulators.
    void EndFrame();

    // Window forwards every Win32 message here via callback.
    void OnMessage(UINT msg, WPARAM wParam, LPARAM lParam);

    // --- Keyboard ---
    bool IsKeyDown(Key key)         const;
    bool IsKeyJustPressed(Key key)  const;
    bool IsKeyJustReleased(Key key) const;

    // --- Mouse ---
    // Absolute cursor position in client coordinates.
    DirectX::XMFLOAT2 GetMousePosition() const { return m_mousePos; }
    // Relative movement this frame via raw input — valid regardless of cursor visibility.
    DirectX::XMFLOAT2 GetMouseDelta()    const { return { float(m_rawDx), float(m_rawDy) }; }
    float              GetMouseWheel()    const { return m_wheelDelta; }
    bool               IsMouseButtonDown(MouseButton btn) const;

    // Hides and clips the cursor to the window for FPS-style look.
    void LockCursor(HWND hwnd);
    void UnlockCursor();
    bool IsCursorLocked() const { return m_cursorLocked; }

    // --- Controller (up to 4, index 0..3) ---
    bool               IsControllerConnected(uint32_t idx = 0) const;
    bool               IsButtonDown(ControllerButton btn, uint32_t idx = 0) const;
    // Normalised [-1, 1] with dead-zone applied.
    DirectX::XMFLOAT2  GetLeftStick(uint32_t idx = 0)   const;
    DirectX::XMFLOAT2  GetRightStick(uint32_t idx = 0)  const;
    // Normalised [0, 1] with threshold applied.
    float              GetLeftTrigger(uint32_t idx = 0)  const;
    float              GetRightTrigger(uint32_t idx = 0) const;

private:
    void  PollControllers();
    static float NormalizeStick(int16_t raw, int16_t deadzone);
    static float NormalizeTrigger(uint8_t raw);

    std::bitset<256> m_keyCurrent;
    std::bitset<256> m_keyPrev;

    DirectX::XMFLOAT2 m_mousePos       = {};
    int32_t           m_rawDx          = 0;
    int32_t           m_rawDy          = 0;
    float             m_wheelDelta     = 0.0f;
    bool              m_mouseButtons[3] = {};

    bool m_cursorLocked = false;

    static constexpr uint32_t k_maxControllers = 4;
    XINPUT_STATE m_ctrlState[k_maxControllers]     = {};
    bool         m_ctrlConnected[k_maxControllers] = {};
};

} // namespace SGE
