#include "scene.h"
#include "path.h"
#include "geometry.h"
#include "spectra.h"
#include "lights.h"
#include "materials.h"
#include "sampler.h"
#include <windows.h>
#undef min
#undef max

static float horizon_dist = 1000.0f;
void scene::isect(path::path_vt init_vt, path* vertex_output, uint32_t tileNdx)
{
    typedef path::path_vt ray;
    ray curr_ray(init_vt);
    bool path_absorbed = false;
    bool path_escaped = false;
    while (!path_absorbed && !path_escaped)
    {
        // Test for intersection
        ray out_vt = curr_ray;
        geometry::vol::vol_nfo* volume_nfo;
        if (geometry::test(curr_ray.dir, &curr_ray.ori, &volume_nfo))
        {
            // Assume solid, filled cubes with no volumetric transmission or surface indents
            // (will involve marching the surface and finding procedural normals; procedural normals are ok, but marching paths through each object's volume grid will be very fiddly & painful)
            // The planar implementation here might be scalable for 3D:
            // https://www.shadertoy.com/view/ltKyzm

            // Somewhat challenging rendering for previs; probably going to do gpu single-bounce/raster (full-screen compute, threads that land on a/the model burrow through a low-res version of its volume and return when they finish),
            // but complicated questions around how much to downres everything
            // might be ok just...not doing that? 1024^3 of model will be a lot to process but its coherent work and its just one model/frame with a single "bounce"
            // previs most-likely won't use full quality otherwise though; leaning on translucent layers (1/1024 alpha each) with flat shading (so just resolving colors for each wavelength sample and temporally accumulating them into
            // a uav), and rays emitted at quarter resolution + upscaled for output

            // Sample surfaces (just lambertian diffuse for now)
            switch (volume_nfo->mat.material_type)
            {
                case material_labels::DIFFUSE:
                    float sample[4];
                    sampler::rand_streams[tileNdx].next(sample);
                    materials::diffuse_surface_sample(&out_vt.dir, &out_vt.pdf, sample[0], sample[1]);
                    materials::diffuse_lambert_reflection(out_vt.rho_sample, volume_nfo->mat.spectral_response, curr_ray.ori);
                    break;
                default:
                    DebugBreak(); // Unsupported material ;_;
                    break;
            }

            // Not complex enough geometry for absorption to happen atm, process it anyways~
            if (curr_ray.rho_weight <= math::eps) path_absorbed = true;
            else
            {
                // Map outgoing direction back from sampling-space to worldspace
                ////////////////////////////////////////////////////////////////

                // Push outgoing rays slightly out of the surface (to help reduce shadow acne)
                out_vt.ori -= out_vt.dir * 0.001f;
                out_vt.mat = &volume_nfo->mat;

                // Cache path vertex for integration
                vertex_output->push(out_vt);

                // Update current ray
                curr_ray = out_vt;
            }
        }
        else
        {
            // Map exiting rays onto the sky (set 1000 units from the scene origin)
            out_vt.ori = curr_ray.dir * lights::sky_dist;

            // Sample our sky model (super wip)
            out_vt.rho_weight = spectra::sky(out_vt.rho_sample, out_vt.dir.e[1]);
            out_vt.rho_weight *= lights::sky_env(&out_vt.pdf);

            // Pass exiting rays into our path buffer
            vertex_output->push(out_vt);
            path_escaped = true;
        }
    }
}
