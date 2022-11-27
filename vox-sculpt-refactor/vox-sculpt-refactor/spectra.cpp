#include "spectra.h"

float spectra::sky(float rho, float y)
{
    const float blue = vmath::clamp(vmath::gaussian(rho, 0.25f, 0.18f, 0.1f, -0.75f), 0.0f, 1.0f);
    const float orange = vmath::clamp(vmath::gaussian(rho, 0.25f, 0.7f, 0.1f, -0.75f), 0.0f, 1.0f);
    return vmath::lerp(blue, orange, vmath::fabs(y));
}

vmath::vec<3> spectra::film(float rho)
{
    return vmath::vec<3>(vmath::max(vmath::quadratic(rho, 4.0f, 0.6f, 0.2f, true), 0.0f) +
                         vmath::max(vmath::quadratic(rho, 4.0f, 3.0f, 1.0f, true), 0.0f),
                         vmath::max(vmath::gaussian(rho, 1.0f, 0.5f, 0.2f, 0.05f), 0.0f),
                         vmath::max(vmath::gaussian(rho, 1.0f, 0.0f, 0.55f, 0.2f) *
                                    vmath::quadratic(rho / 0.4f, 1.0f, -0.6f / 0.4f, -2.3f, false) *
                                    vmath::quadratic(rho, 1.0f, 0.95f, 0.0f, false) + 0.1f, 0.0f));
}

const float spectra::placeholder_spd(vmath::vec<4> coords_and_rho)
{
    return vmath::gaussian(coords_and_rho.w(), 2.7f, 0.8f, 0.2f, 1.7f); // Debug red :p
}

void spectra::spectral_buckets::init()
{
    // Random bucket selection by default
    for (uint32_t i = 0; i < num_buckets; i++)
    {
        buckets[i] = 0.0f;
    }
}

float spectra::spectral_buckets::draw_sample(float u, float v)
{
    // Try to select a frequency from a curve skewed towards colors reflected most by the environment
    // This should work for most scenes
    for (uint32_t n = 0; n < num_buckets; n++)
    {
        if (u < buckets[n]) // Buckets with higher sensitivities will naturally pass this test more often over time
        {
            last_bucket = n; // Record the selected bucket so we can update it with a weight after tracing
            return (n * interval_size) + (v * interval_size); // Return a random offset within the selected bucket
        }
    }

    // The surface is matte black; all frequencies reflected equally poorly
    // Since no frequencies are particularly bright, just return one at random
    last_bucket = static_cast<uint32_t>(vmath::ffloor(u * num_buckets));
    return (last_bucket * interval_size) + (v * interval_size);
}

void spectra::spectral_buckets::update(float weight)
{
    buckets[last_bucket] = weight; // Any noise from using per-sample weights seems to be cancelled out during filtering, and
                                   // naively smoothing here (e.g. by using a weight / num_samples box filter) seems to damage the
                                   // importance function (by skewing weights down) more than it helps account for natural spectral
                                   // variance
}
