
#include "vmath.h"
#include "materials.h"

namespace materials
{
    void diffuse_lambert_reflection(float rho, vmath::fn<4, const float> surf_spd, vmath::vec<3> coords, float* power, float* rho_weight)
    {
        *rho_weight *= surf_spd.invoke(vmath::vec<4>(coords.x(), coords.y(), coords.z(), rho));
        *power *= vmath::inv_pi;
    }

    void diffuse_surface_sample(vmath::vec<3>* out_dir, float* out_pdf, float rnd_u, float rnd_v)
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
            bool uGreater = vmath::fabs(rnd_u) > vmath::fabs(rnd_v);
            float r = uGreater ? rnd_u : rnd_v;
            float theta = uGreater ? vmath::quarter_pi * (rnd_u / rnd_v) :
                vmath::half_pi * (rnd_v / rnd_u);
            rnd_u = r * static_cast<float>(cos(theta));
            rnd_v = r * static_cast<float>(sin(theta));
        }
        rnd_u *= rnd_u;
        rnd_v *= rnd_v;
        *out_dir = vmath::vec<3>(rnd_u, static_cast<float>(sqrt(1.0f - (rnd_u + rnd_v))), rnd_v).normalized();
        *out_pdf = out_dir->dot(vmath::vec<3>(0, 1, 0)) / vmath::pi; // generated ray is in sampling space and sees the normal parallel to y-up
        // nDotL *and* div by pi feels super sketchy
        // ~Fairly sure this evaluates down to just the y-axis of the sampled ray, divided by pi;
        // kinda feel like testing to make sure before optimizing
        // need to review pdf here against PBRT
    }
};