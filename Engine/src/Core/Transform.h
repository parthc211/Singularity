#pragma once

#include <DirectXMath.h>

namespace SGE {

struct Transform
{
    DirectX::XMFLOAT3 position = { 0.0f, 0.0f, 0.0f };
    DirectX::XMFLOAT3 rotation = { 0.0f, 0.0f, 0.0f }; // pitch, yaw, roll in radians
    DirectX::XMFLOAT3 scale    = { 1.0f, 1.0f, 1.0f };

    DirectX::XMMATRIX GetMatrix() const
    {
        using namespace DirectX;
        return XMMatrixScaling(scale.x, scale.y, scale.z)
             * XMMatrixRotationRollPitchYaw(rotation.x, rotation.y, rotation.z)
             * XMMatrixTranslation(position.x, position.y, position.z);
    }
};

} // namespace SGE
