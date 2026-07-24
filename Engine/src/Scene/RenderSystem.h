#pragma once
#include <cstdint>

struct ID3D12GraphicsCommandList; // COM interface, forward-declared

namespace SGE {

class World;
class Camera;
class DynamicUploadBuffer;

// Draws every entity that has TransformComponent + MeshComponent. For each:
//   1. compute MVP = model * viewProj (CPU side),
//   2. write { MVP, Model, BaseColor } into the per-frame upload arena,
//   3. bind that block as a root CBV (b0),
//   4. record the mesh's indexed draw.
//
// rootParamIndexCBV is the *root-parameter slot* of your b0 root CBV (the slot
// index you'd pass to SetGraphicsRootConstantBufferView), not the register.
class RenderSystem {
public:
    void Render(World& world,
                const Camera& camera,
                DynamicUploadBuffer& objectCB,
                ID3D12GraphicsCommandList* cmd,
                uint32_t rootParamIndexCBV);
};

} // namespace SGE
