#pragma once

#include "vmath.h"
#include "spectra.h"

// Supported materials, mostly planned (only lambertian diffuse implemented atm)
// Material properties will eventually be data-driven from imgui (via d3d11), not hardcoded
enum class material_labels
{
    DIFFUSE, // Currently lambertian, oren-nayar planned
    SPECULAR_CONDUCTOR, // Spectral smith/ggx
    SPECULAR_DIELECTRIC, // Spectral smith/ggx

    // Need to read PBR volumetrics + catch up on papers for these
    VOLUMETRIC_CLOUDY, // Like dust or vapour, particles in air; not sure yet what model for this one
    VOLUMETRIC_DENSE, // Particles in water or another dense fluid, like honey
    SUBSURFACE, // Transmission & reflection through organic volumes, like skin or vegetation
    HAIR, // Specialized scattering for human hair
    FUR // Specialized scattering for animal fur
};

namespace materials
{
    // Lambertian surfaces have constant reflection and no other spectrally-varying properties, so our spd just needs to take
    // the current sampled frequency + worldspace coordinates (for spatially-varying colours)
    void diffuse_lambert_reflection(float rho, vmath::fn<4, const float> surf_spd, vmath::vec<3> coords, float* power, float* rho_weight);

    // Cosine hemisphere sampling with Malley's Method;
    // implemented from the cosine-weighted hemisphere sampling
    // strategy shown in
    // Physically Based Rendering: From Theory to Implementation
    // (Pharr, Jakob, Humphreys)
    // https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#Cosine-WeightedHemisphereSampling
    // This also implements the PBR concentric disk mapping technique, which is described here:
    // https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#SamplingaUnitDisk
    void diffuse_surface_sample(vmath::vec<3>* out_dir, float* out_pdf, float rnd_u, float rnd_v);

    // A concrete material instance, allowing specific objects to bind themselves to the sampling/shading functions specified above
    struct instance
    {
        material_labels material_type = material_labels::DIFFUSE;
        vmath::fn<4, const float> spectral_response; // 4D spatially-varying color response function (arguments in order are wavelength then x/y/z)
        vmath::fn<4, const float> spectral_ior; // Spatially-varying index-of-refraction, expressed as a response function for different wavelengths
        // ...additional curves for volumetric materials go here...
        float roughness = 0.0f; // Slightly different meaning for this between different material classes (oren-nayar, ggx/smith, microflake volumes...)
    };
};