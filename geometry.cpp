#include "geometry.h"
#include "mem.h"
#include <ranges>

geometry::tri* geometry::triangles;
void geometry::init()
{
   // Allocate triangle memory
   triangles = mem::allocate_tracing<math::m3>(geometry_footprint);

   // Just one triangle for now, equilateral and ~five units from the camera
   // (mesh import code will go here in future)
   triangles[0] = math::m3(math::v3(-1.0f, 0, 5.0f),
                           math::v3(1.0f, 0, 5.0f),
                           math::v3(0.5f, 0, 5.0f));
}

bool geometry::test(math::v3 dir, math::v3* ori_inout, material_labels* mat_intersected_out, math::v3* bary_coords_out, math::v3* normal_out)
{
   // Trashy loop disregards depth and visibility; future versions will perform BVH traversal here
   for (uint32_t t = 0; t < numTriangles; t++)
   {
      // Muller-Trumbore :D
      // Ran out of time to implement this :(
      // Likely to follow the explanation given here:
      // https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   }
   return false;
}
