#pragma once

#include <DirectXMath.h>

namespace SGE {

class Camera
{
public:
    void LookAt(DirectX::XMFLOAT3 eye, DirectX::XMFLOAT3 target,
                DirectX::XMFLOAT3 up = { 0.0f, 1.0f, 0.0f })
    {
        using namespace DirectX;
        XMMATRIX m = XMMatrixLookAtLH(XMLoadFloat3(&eye), XMLoadFloat3(&target), XMLoadFloat3(&up));
        XMStoreFloat4x4(&m_view, m);
    }

    void Perspective(float fovYRadians, float aspect, float nearZ, float farZ)
    {
        DirectX::XMMATRIX m = DirectX::XMMatrixPerspectiveFovLH(fovYRadians, aspect, nearZ, farZ);
        DirectX::XMStoreFloat4x4(&m_proj, m);
    }

    DirectX::XMMATRIX GetView()           const { return DirectX::XMLoadFloat4x4(&m_view); }
    DirectX::XMMATRIX GetProjection()     const { return DirectX::XMLoadFloat4x4(&m_proj); }
    DirectX::XMMATRIX GetViewProjection() const { return GetView() * GetProjection(); }

private:
    DirectX::XMFLOAT4X4 m_view = {};
    DirectX::XMFLOAT4X4 m_proj = {};
};

} // namespace SGE
