#pragma once

#include "math.h"
#include <stdint.h>

class tracing
{
public:
   struct ray
   {
      math::v3 dir;
      math::v3 ori;
   };

   static void trace(int16_t width, int16_t height, int16_t minX, int16_t minY, int16_t tileNdx);
   static void stop_tracing();
};