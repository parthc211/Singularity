#include "RenderSystem.h"
#include "World.h"
#include "Components.h"
#include "../Core/Camera.h"                       // Camera::GetViewProjection()
#include "../Renderer/Mesh.h"                      // Mesh::Draw(cmd)
#include "../Renderer/DX12/DynamicUploadBuffer.h"
#include <d3d12.h>
#include <DirectXMath.h>
#include <cstring>

using namespace DirectX;

namespace SGE {

// Must match the cbuffer layout in Triangle.hlsl. Stored row-major; HLSL reads
// it column-major, which is the implicit transpose the engine relies on (see
// PLAN.md "MVP convention"). So: NO XMMatrixTranspose here.
struct ObjectConstants {
    XMFLOAT4X4 MVP;
    XMFLOAT4X4 Model;
    XMFLOAT4   BaseColor;
};

void RenderSystem::Render(World& world,
                          const Camera& camera,
                          DynamicUploadBuffer& objectCB,
                          ID3D12GraphicsCommandList* cmd,
                          uint32_t rootParamIndexCBV) {
    const XMMATRIX viewProj = camera.GetViewProjection();

    // MeshComponent listed first: it drives iteration (renderables are the
    // smaller set), and we filter to those that also have a TransformComponent.
    world.View<MeshComponent, TransformComponent>(
        [&](Entity e, MeshComponent& mc, TransformComponent& tc) {
            if (!mc.MeshPtr) return;

            const XMMATRIX model = tc.GetMatrix();

            ObjectConstants oc;
            XMStoreFloat4x4(&oc.MVP,   model * viewProj); // row-vector order
            XMStoreFloat4x4(&oc.Model, model);
            if (MaterialComponent* mat = world.TryGet<MaterialComponent>(e))
                oc.BaseColor = mat->BaseColor;
            else
                oc.BaseColor = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);

            const auto alloc = objectCB.Allocate(sizeof(ObjectConstants));
            if (!alloc.Cpu) return; // arena exhausted this frame
            std::memcpy(alloc.Cpu, &oc, sizeof(ObjectConstants));

            cmd->SetGraphicsRootConstantBufferView(rootParamIndexCBV, alloc.Gpu);
            mc.MeshPtr->Draw(cmd);
        });
}

} // namespace SGE
