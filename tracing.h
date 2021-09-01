#pragma once

#include "math.h"
#include <stdint.h>
#include "path.h"

class tracing
{
public:
   struct ray
   {
      math::vec<3> dir;
      math::vec<3> ori;
   };
   static path* cameraPaths; // One reusable path/tile for now, minx * miny expected for VCM
                             // (so we can process each one multiple times against arbitrary light paths)
   static float* isosurf_distances; // Distances to sculpture boundaries from grid bounds, per-subpixel, refreshed on camera zoom/rotate + animation timesteps (if/when I decide to implement those)

   //static path lightPaths[parallel::numTiles];
   static constexpr uint32_t path_footprint = path::capacity * sizeof(path::path_vt);
   static void trace(int16_t width, int16_t height, int16_t minX, int16_t minY, int16_t tileNdx);
   static void stop_tracing();
   static void init();

};