#pragma once

#include "vmath.h"

namespace spectra
{
    // Super basic sky model - sharp blue fading to orange as y approaches zero
    // Desmos visualization
    // https://www.desmos.com/calculator/7qqnkr1uqx
    float sky(float rho, float y);

    // Custom film/sensor response curve, kinda follows the references in Zucconi's diffraction tutorial but heavily iterated to give a more film-like
    // spectrum; Zucconi's tutorial is here
    // https://www.alanzucconi.com/2017/07/15/improving-the-rainbow/,
    // aand my Desmos workspace for the curves themselves is here:
    // https://www.desmos.com/calculator/zm9jwr0ngk
    vmath::vec<3> film(float rho);

    // Generic diffuse surface colors until I get imgui working
    const float placeholder_spd(vmath::vec<4> coords_and_rho); // [xyz], w (wavelength)

    // Stratified spectral representation for camera sampling
    struct spectral_buckets
    {
        static constexpr uint32_t num_buckets = 16;
        static constexpr float interval_size = 1.0f / num_buckets;
        uint32_t last_bucket = 0;
        float buckets[num_buckets] = {};
        void init();
        float draw_sample(float u, float v);
        void update(float weight);
    };
};
