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
bool LoadOBJ(const char*               path,
             std::vector<MeshVertex>&  outVertices,
             std::vector<uint32_t>&    outIndices);

} // namespace SGE
