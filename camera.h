#pragma once

#include "ui.h"
#include "mem.h"
#include "tracing.h"
#include "math.h"
#include "scene.h"
#include "spectra.h"
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

   // Intermediate sensor values for filtering/anti-aliasing/temporal integration
   static sensel* sensor_grid;
   static constexpr uint32_t sensor_grid_footprint = ui::window_width * ui::window_height * sizeof(sensel);

   // Screen color/picture data (8bpc)
   static RGBQUAD* digital_colors;
   static constexpr uint32_t digital_colors_footprint = ui::window_width * ui::window_height * sizeof(RGBQUAD);

   // Expected memory footprint for this system
   static constexpr uint32_t max_footprint = sensor_grid_footprint +
                                             digital_colors_footprint;

   // Camera sampling! just perspective projection for now :)
   static inline const math::v3 camera_pos() { return math::v3(0, 0, -10.0f); }
   static constexpr float FOV_RADS = math::pi * 0.5f;
   static path::path_vt lens_sample(float film_x, float film_y, float rho)
   {
      const math::v3 c = camera_pos();
      return path::path_vt(math::v3(film_x - ui::image_centre_x,
                                    film_y - ui::image_centre_y,
                                    ui::window_width / tan(FOV_RADS * 0.5f)).normalized(), // Probably don't need to normalize here, but the stability feels nice
                             c, 1.0f, rho, 1.0f); // No colour-cast on exiting rays atm
   }

   // Combination of a custom sensor response curve (see spectra.h) and a basic integration scheme
   //
   // Integration scheme is very simple for now, just adding colors together and dividing by
   // the total number of samples; future versions will use a more sophisticated filter with no
   // decay (so late samples will be implicitly weighted the same as early ones) and I might
   // eventually consider performing integration in spectral space instead (not 100% sure what
   // that would look like)
   static void sensor_response(float rho, float weight, uint32_t ndx, uint32_t sample_num)
   {
      // Resolve responses per-channel
      math::v3 rgb = spectra::film(rho);

      // Compose isolated colours into a sensor value, apply accumulated weights
      // (from path-tracing + spectral integration), write to sensor output :)
      sensel& curr_sensel = sensor_grid[ndx]; // Read in current sensor value
      weight *= 1.0f / 4.0f; // Apply sample weighting into path weight
      weight *= (sample_num > 4) ? 0.0f : 1.0f; // Super-basic filter scheme, future versions will adapt the filter/integration code I wrote for Athru
      if (sample_num > 1 && sample_num < 4)
      {
         curr_sensel.r += rgb.x * weight;
         curr_sensel.g += rgb.y * weight;
         curr_sensel.b += rgb.z * weight;
      }
      else if (sample_num == 1)
      {
         curr_sensel.r = rgb.x * weight;
         curr_sensel.g = rgb.y * weight;
         curr_sensel.b = rgb.z * weight;
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
      sensor_grid = mem::allocate_tracing<sensel>(sensor_grid_footprint);
      digital_colors = mem::allocate_tracing<RGBQUAD>(digital_colors_footprint);
   }
};