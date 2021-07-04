#pragma once

#include "math.h"
#include "spectra.h"
#include "functional"

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

class materials
{
public:
    // Lambertian surfaces have constant reflection and no other spectrally-varying properties, so our spd just needs to take
    // the current sampled frequency + barycentric coordinates (for spatially-varying colours)
    static float diffuse_lambert_reflection(float rho, math::fn<4, const float> surf_spd, math::vec<3> coords)
    {
        return surf_spd.invoke(math::vec<4>(coords.x(), coords.y(), coords.z(), rho)) * math::inv_pi;
    }

    // Cosine hemisphere sampling with Malley's Method;
    // implemented from the cosine-weighted hemisphere sampling
    // strategy shown in
    // Physically Based Rendering: From Theory to Implementation
    // (Pharr, Jakob, Humphreys)
    // https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#Cosine-WeightedHemisphereSampling
    // This also implements the PBR concentric disk mapping technique, which is described here:
    // https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#SamplingaUnitDisk
    static void diffuse_surface_sample(math::vec<3>* out_dir, float* out_pdf, float rnd_u, float rnd_v)
    {
        // Map generated random values onto a unit square
        // centered at the origin (occupying the domain
        // [-1.0f...1.0f]^2)
        rnd_u = (rnd_u * 2.0f) - 1.0f;
        rnd_v = (rnd_v * 2.0f) - 1.0f;

        // This is a mapping between incompatible spaces
        // (specifically, a square fold over a disc), so
        // applying the mapping to the centre of the
        // projection will send input points to infinity;
        // avoid that by branching here (assumes the
        // centre of the projection is at [0.0f.elts[0]x])
        if (rnd_u != 0.0f &&
            rnd_v != 0.0f)
        {
            // We know the mapping won't send [uv] to infinity, so
            // we can safely map points from [-1.0f...1.0f]^2 onto
            // the disc without generating any artifacts

            // Infer radial distance from the larger absolute value between
            // [uv.elts[0]] and [uv.elts[1]], then re-apply the sign of the original
            // axis
            bool uGreater = abs(rnd_u) > abs(rnd_v);
            float r = uGreater ? rnd_u : rnd_v;
            float theta = uGreater ? math::quarter_pi * (rnd_u / rnd_v) :
                                     math::half_pi * (rnd_v / rnd_u);
            rnd_u = r * cos(theta);
            rnd_v = r * sin(theta);
        }
        rnd_u *= rnd_u;
        rnd_v *= rnd_v;
        *out_dir = math::vec<3>(rnd_u, sqrt(1.0f - (rnd_u + rnd_v)), rnd_v).normalized();
        *out_pdf = out_dir->dot(math::vec<3>(0, 1, 0)) / math::pi; // generated ray is in sampling space and sees the normal parallel to y-up
                                                                   // ~Fairly sure this evaluates down to just the y-axis of the sampled ray, divided by pi;
                                                                   // kinda feel like testing to make sure before optimizing
    }

    // A concrete material instance, allowing specific objects to bind themselves to the sampling/shading functions specified above
    struct instance
    {
        material_labels material_type = material_labels::DIFFUSE;
        math::fn<4, const float> spectral_response; // 4D spatially-varying color response function (arguments in order are wavelength then x/y/z)
        math::fn<4, const float> spectral_ior; // Spatially-varying index-of-refraction, expressed as a response function for different wavelengths
        // ...additional curves for volumetric materials go here...
        float roughness = 0.0f; // Slightly different meaning for this between different material classes (oren-nayar, ggx/smith, microflake volumes...)
    };
};