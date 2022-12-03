#include "lights.h"

float lights::sky_env(float* pdf_out)
{
    *pdf_out = (vmath::inv_pi * 0.25f); // Distantly remembered sphere pdf (1 / 4pi), not totally sure if its appropriate here
    return (1.0f / (sky_dist * sky_dist)) * sky_brightness;
}
