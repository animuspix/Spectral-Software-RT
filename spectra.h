#pragma once

#include "math.h"

class spectra
{
   public:
   // Super basic sky model - sharp blue fading to orange as y approaches zero
   static float sky(float rho, float y)
   {
      float blue = 1.0f - std::abs(0.2f - rho);
      blue *= blue;
      float orange = 1.0f - std::abs(0.7f - rho);
      return std::lerp(blue, orange, y);
   }

   // Custom film/sensor response curve, kinda follows the references in Zucconi's diffraction tutorial but heavily iterated to give a more film-like
   // spectrum; Zucconi's tutorial is here
   // https://www.alanzucconi.com/2017/07/15/improving-the-rainbow/,
   // aand my Desmos workspace for the curves themselves is here:
   // https://www.desmos.com/calculator/zm9jwr0ngk
   static math::v3 film(float rho)
   {
      return math::v3(std::max(math::quadratic(rho, 4.0f, 0.6f, 0.2f), 0.0f) +
                      std::max(math::quadratic(rho, 4.0f, 3.0f, 1.0f), 0.0f),
                      std::max(math::gaussian(rho, 1.0f, 0.5f, 0.2f, 0.05f), 0.0f),
                      std::max(math::gaussian(rho, 1.0f, 0.0f, 0.55f, 0.2f) *
                               math::quadratic(rho / 0.4f, 1.0f, -0.6f / 0.4f, -2.3f, false) *
                               math::quadratic(rho, 1.0f, 0.95f, 0.0f, false) + 0.1f, 0.0f));
   }

   // Generic diffuse surface colors until I get imgui working
   static float placeholder_spd(float rho, math::v3 bary_coords)
   {
      return sin(rho) / bary_coords.magnitude(); // Stripes :D
                                                 // Stripes will fade to blue (eventually black) the further they get from the origin of the current triangle
                                                 // (not expecting to support any other geometries)
   }
};
