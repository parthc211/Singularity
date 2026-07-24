#pragma once
#include <DirectXMath.h>

namespace SGE {

class Mesh; // forward-declared: MeshComponent only stores a pointer, so we
            // don't pull the DX12-heavy Mesh.h into everything that sees a World.

// Position / Rotation (euler radians) / Scale -> world matrix.
// Self-contained (doesn't depend on your Core/Transform.h) so it compiles
// regardless of that type's exact API; swap GetMatrix() to call your Transform
// if you'd rather have one source of truth.
struct TransformComponent {
    DirectX::XMFLOAT3 Position{ 0.0f, 0.0f, 0.0f };
    DirectX::XMFLOAT3 Rotation{ 0.0f, 0.0f, 0.0f }; // euler radians (pitch, yaw, roll)
    DirectX::XMFLOAT3 Scale   { 1.0f, 1.0f, 1.0f };

    // Physics writes orientation as a quaternion (x,y,z,w) — eulers can't
    // represent an integrated angular velocity without gimbal artifacts. The
    // flag keeps the euler path the default so the 12 pre-physics scenes are
    // untouched; PhysicsSystem::SyncTransforms flips it per entity.
    DirectX::XMFLOAT4 RotationQuat{ 0.0f, 0.0f, 0.0f, 1.0f };
    bool UseQuaternion = false;

    DirectX::XMMATRIX GetMatrix() const {
        using namespace DirectX;
        // Row-vector convention (matches the engine): point' = point * S*R*T,
        // i.e. scale, then rotate, then translate.
        const XMMATRIX rot = UseQuaternion
            ? XMMatrixRotationQuaternion(XMLoadFloat4(&RotationQuat))
            : XMMatrixRotationRollPitchYaw(Rotation.x, Rotation.y, Rotation.z);
        return XMMatrixScaling(Scale.x, Scale.y, Scale.z)
             * rot
             * XMMatrixTranslation(Position.x, Position.y, Position.z);
    }
};

// Non-owning. The referenced Mesh must outlive the entity (own your meshes in
// the app / an asset manager, not in the component).
struct MeshComponent {
    Mesh* MeshPtr = nullptr;
};

// Minimal for now — a tint the pixel shader multiplies in. Grows later into
// texture handles, a PSO override, PBR parameters, etc.
struct MaterialComponent {
    DirectX::XMFLOAT4 BaseColor{ 1.0f, 1.0f, 1.0f, 1.0f };
};

} // namespace SGE
