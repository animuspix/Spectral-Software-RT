#pragma once

#include "math.h"

class lights
{
   public:
   static constexpr float sky_dist = 1000.0f; // Sky is assumed to be ~1000 units away from everything in the scene
   static constexpr float sky_brightness = 250000000.0f; // Totally nonphysical number, adjusted experimentally
   static float sky_env(float* pdf_out) // Returns sky intensity from the given ray origin; doesn't perform a visibility test (yet), so not useful
                                        // for MIS/next-event-estimation
   {
      *pdf_out = (math::inv_pi  * 0.25f); // Distantly remembered sphere pdf (1 / 4pi), not totally sure if its appropriate here
      return (1.0f / (sky_dist * sky_dist)) * sky_brightness;
   }
};