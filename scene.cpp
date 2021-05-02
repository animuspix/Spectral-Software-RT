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
bool path_absorbed = true;
bool path_escaped = false;
void scene::isect(path::path_vt init_vt, path* vertex_output, uint32_t tileNdx)
{
   typedef path::path_vt ray;
   ray curr_ray(init_vt);
   while (!path_absorbed && !path_escaped)
   {
      // Test for intersection
      ray out_vt = curr_ray;
      material_labels mat;
      math::v3 bary(0, 0, 0);
      math::v3 normal(0, 0, 0);
      if (geometry::test(curr_ray.dir, &curr_ray.ori, &mat, &bary, &normal))
      {
         // Sample surfaces (just lambertian diffuse for now)
         // Colors but no shading (resampling) because tiiiiime
         // Future versions will
         switch (mat)
         {
            case material_labels::DIFFUSE:
               // Don't strictly need to compute omega_o here since lambertian reflection is isotropic, but we will for oren-nayar so do it anyway :p
               float sample[4];
               sampler::rand_streams[tileNdx].next(sample);
               materials::diffuse_surface_sample(&out_vt, sample[0], sample[1]);
               materials::diffuse_lambert_reflection(out_vt.rho_sample, [](float rho_sample, math::v3 bary_coords) { return spectra::placeholder_spd(rho_sample, bary_coords); }, bary);
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
            out_vt.dir += out_vt.dir * 0.001f;

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
         out_vt.rho_weight = spectra::sky(out_vt.rho_sample, out_vt.dir.y);
         out_vt.rho_weight *= lights::sky_env(&out_vt.pdf);

         // Pass exiting rays into our path buffer
         vertex_output->push(out_vt);
         path_escaped = true;
      }
   }
}
