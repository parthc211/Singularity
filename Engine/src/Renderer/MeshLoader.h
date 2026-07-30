#pragma once

#include "Renderer/Mesh.h"

#include <vector>
#include <cstdint>

namespace SGE {

// Parses a Wavefront OBJ file into flat vertex and index arrays ready for GPU upload.
//
// Behaviour:
//   - Handles v / vn / vt / f tokens; all others (o, g, mtllib, usemtl, s) are ignored.
//   - Face vertices are de-duplicated by unique (position, normal, texCoord) combination.
//   - Polygon faces are fan-triangulated (vertex 0, i, i+1).
//   - OBJ negative indices (relative from end of list) are resolved correctly.
//   - Missing normals or UVs in a face vertex produce zero-valued fields.
//
// Returns false if the file cannot be opened or contains no geometry.
//   - Tangents are computed automatically (ComputeTangents) when the file has UVs.
bool LoadOBJ(const char*               path,
             std::vector<MeshVertex>&  outVertices,
             std::vector<uint32_t>&    outIndices);

// Fills each vertex's tangent from its position/normal/texCoord and the triangle
// list (Lengyel's method): per-triangle tangents/bitangents from UV deltas are
// area-accumulated per vertex, then Gram-Schmidt-orthogonalized against the
// normal; w stores the handedness (±1) so shaders can rebuild the bitangent as
// cross(N, T) * w. Triangles with degenerate UVs contribute nothing; vertices
// that end up with no usable tangent get an arbitrary one perpendicular to
// their normal (flat normal maps still shade correctly there).
void ComputeTangents(std::vector<MeshVertex>&     vertices,
                     const std::vector<uint32_t>& indices);

} // namespace SGE
