#pragma once

#include "ui.h"
#include "mem.h"
#include "tracing.h"
#include "math.h"
#include <assert.h>
#include <algorithm>

class camera
{
public:
   // Digital color/sensor data model, used to map from spectral inputs to writable screen colors
   // (slowly turning into a retinal cone cell model instead oops, might update names eventually)
   struct sensel
   {
      sensel(float _r, float _g, float _b) :
         r(_r), g(_g), b(_b) {}
      sensel operator+(const sensel& rhs)
      {
         return sensel(rhs.r + r, rhs.g + g, rhs.b + b);
      }
      sensel operator*(const float rhs) const
      {
         return sensel(r * rhs, g * rhs, b * rhs);
      }
      float r;
      float g;
      float b;
   };

   // Analogue color/spectral data
   struct spectrum
   {
      spectrum(float _rho, float _weight) : rho(_rho), weight(_weight) {}
      float rho;
      float weight = 1.0f; // Different spectral samples may be filtered in different ways (because of actual color filters with a response curve,
                           // or because of placement on the lens, diffraction/depth-of-field, etc)
   };
   static spectrum* analog_colors;
   static constexpr uint32_t analog_colors_footprint = ui::window_width * ui::window_height * sizeof(spectrum);

   // Intermediate sensor values for filtering/anti-aliasing/temporal integration
   static sensel* sensor_grid;
   static constexpr uint32_t sensor_grid_footprint = ui::window_width * ui::window_height * sizeof(sensel);

   // Screen color/picture data (8bpc)
   static RGBQUAD* digital_colors;
   static constexpr uint32_t digital_colors_footprint = ui::window_width * ui::window_height * sizeof(RGBQUAD);

   // Expected memory footprint for this system
   static constexpr uint32_t max_footprint = analog_colors_footprint +
                                             sensor_grid_footprint +
                                             digital_colors_footprint;

   // Combination of a custom sensor response curve and a basic integration scheme
   //
   // Custom film/sensor response curve kinda follows the references in Zucconi's diffraction tutorial, but heavily iterated to give a more film-like
   // spectrum; Zucconi's tutorial is here
   // https://www.alanzucconi.com/2017/07/15/improving-the-rainbow/,
   // aand my Desmos workspace for the curves themselves is here:
   // https://www.desmos.com/calculator/hz6yrxynho
   //
   // Integration scheme is very simple for now, just adding colors together and dividing by
   // the total number of samples; future versions will use a more sophisticated filter with no
   // decay (so late samples will be implicitly weighted the same as early ones) and I might
   // eventually consider performing integration in spectral space instead (not 100% sure what
   // that would look like)
   static void sensor_response(spectrum s, uint32_t ndx, uint32_t sample_num)
   {
      // Resolve responses per-channel
      // (CIE XYZ full-gamut color)
      float sRed = max(math::quadratic(s.rho, 4.0f, 0.6f, 0.2f), 0.0f) +
                   max(math::quadratic(s.rho, 4.0f, 3.0f, 1.0f), 0.0f);
      float sGreen = max(math::gaussian(s.rho, 1.0f, 0.5f, 0.2f, 0.05f), 0.0f);
      float sBlue = max(math::gaussian(s.rho, 1.0f, 0.0f, 0.55f, 0.2f) *
                        math::quadratic(s.rho / 0.4f, 1.0f, -0.6f / 0.4f, -2.3f, false) *
                        math::quadratic(s.rho, 1.0f, 0.95f, 0.0f, false) + 0.1f, 0.0f);

      // Compose isolated colours into a sensor value, apply accumulated weights
      // (from path-tracing + spectral integration), write to sensor output :)
      sensel& curr_sensel = sensor_grid[ndx]; // Read in current sensor value
      s.weight *= 1.0f / 4.0f; // Apply sample weighting into path weight
      s.weight *= (sample_num > 4) ? 0.0f : 1.0f;
      if (sample_num > 1 && sample_num < 4)
      {
         curr_sensel.r += sRed * s.weight;
         curr_sensel.g += sGreen * s.weight;
         curr_sensel.b += sBlue * s.weight;
      }
      else if (sample_num == 1)
      {
         curr_sensel.r = sRed * s.weight;
         curr_sensel.g = sGreen * s.weight;
         curr_sensel.b = sBlue * s.weight;
      }
   }

   // Tonemap + write to digital output :)
   static void tonemap_out(uint32_t ndx)
   {
      // Tonemap into integer 8bpc + return
      // Tonemapped with ACES, source:
      // https://www.shadertoy.com/view/WdjSW3
      // originally Narkowicz 2015, "ACES Filmic Tone Mapping Curve"
      auto aces = [](float x) {
         const float a = 2.51;
         const float b = 0.03;
         const float c = 2.43;
         const float d = 0.59;
         const float e = 0.14;
         return (x * (a * x + b)) / (x * (c * x + d) + e);
      };
      sensel sensor_v = sensor_grid[ndx];
      RGBQUAD out;
      out.rgbRed = (BYTE)(aces(std::clamp(sensor_v.r, 0.0f, 1.0f)) * 256);
      out.rgbGreen = (BYTE)(aces(std::clamp(sensor_v.g, 0.0f, 1.0f)) * 256);
      out.rgbBlue = (BYTE)(aces(std::clamp(sensor_v.b, 0.0f, 1.0f)) * 256);
      out.rgbReserved = 0;
      digital_colors[ndx] = out;
   }

   // Initialize
   static void init()
   {
      assert(mem::tracing_arena != nullptr);
      analog_colors = (spectrum*)mem::tracing_arena;
      sensor_grid = (sensel*)((uint8_t*)analog_colors + analog_colors_footprint);
      digital_colors = (RGBQUAD*)((uint8_t*)sensor_grid + sensor_grid_footprint);
   }
};