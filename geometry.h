#pragma once

#include "math.h"
#include "materials.h"

class geometry
{
   public:
   typedef math::m3 tri;
   static tri* triangles; // Eventually using a dynamic array for this, adding an actual allocator to my memory system first for better scalability
   static constexpr uint32_t numTriangles = 1;
   static constexpr uint32_t geometry_footprint = numTriangles * sizeof(tri);
   static void init();
   static bool test(math::v3 dir, math::v3* ori_inout, material_labels* mat_intersected_out, math::v3* bary_coords_out, math::v3* normal_out);
};

