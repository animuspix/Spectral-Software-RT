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
    uint8_t bounceCtr = 0;
    while (!path_absorbed && !path_escaped/* && bounceCtr < path::capacity*/)
    {
        // Test for intersection
        ray out_vt = curr_ray;
        geometry::vol::vol_nfo* volume_nfo;
        if (geometry::test(curr_ray.dir, &curr_ray.ori, &volume_nfo))
        {
            // Resolve intersection cell & normal; no handling for sparse volumes atm, considering a combination of occupancy bits and euclidean surface distances in 0...128 (cell units)
            // (replacing edge fuzz, and allowing easy editing + efficient SDF marching while keeping one byte/cell)
            ////////////////////////////////////////

            // Resolve intersection cell
            math::vec<3> rel_p = (curr_ray.ori - volume_nfo->pos) + (volume_nfo->scale * 0.5f); // Relative position from lower object corner
            math::vec<3> uvw = rel_p / volume_nfo->scale; // Normalized UVW
            math::vec<3> uvw_scaled = uvw * geometry::vol::width; // Voxel coordinates! :D
            math::vec<3> uvw_i = math::floor(uvw_scaled);
#ifdef _DEBUG
            assert(uvw_i.x() >= 0.0f && uvw_i.y() >= 0.0f && uvw_i.z() >= 0.0f); // Voxel coordinates should never be negative
#endif
            // Marching for final intersection will happen here; existing code treats every volume as a homogenous cuboid solid
            ////////////////////////////////////////

            // Compute normal; resolve cell offset, translate it relative to the cell centre instead of the lower corner, resolve the smallest axis, then zero it;
            // each of the remaining two axes represent a candidate normal, so pick the larger (closer) one if they're not equal (the current cell offset is clearly
            // on a specific face of the current voxel), and take whichever one does not point into an occluded cell if they are (since solving normals this way produces
            // singularities along voxel edges, and naively using the largest axis leads to rays being trapped when normals chosen at the edges are coplanar with the larger
            // surface being rendered)
            math::vec<3> cellOffs = (uvw_scaled - uvw_i) - math::vec<3>(0.5f, 0.5f, 0.5f);
            math::vec<3> absCellOffs = math::abs(cellOffs);
            const float min_elt = std::min(std::min(absCellOffs.x(), absCellOffs.y()), absCellOffs.z());
            uint8_t min_axis = (min_elt >= (absCellOffs.x() - math::eps) && min_elt <= (absCellOffs.x() + math::eps)) ? 0 :
                               (min_elt >= (absCellOffs.y() - math::eps) && min_elt <= (absCellOffs.y() + math::eps)) ? 1 :
                               (min_elt >= (absCellOffs.z() - math::eps) && min_elt <= (absCellOffs.z() + math::eps)) ? 2 : 2; // If all axes are roughly equal (rare but possible) assume v ~= z
            math::vec<3> n = cellOffs;
            n.e[min_axis] = 0;
            uint8_t candidate_0 = min_axis > 0 ? min_axis - 1 : 2;
            uint8_t candidate_1 = min_axis < 2 ? min_axis + 1 : 0;
            if (abs(n.e[candidate_0]) < (abs(n.e[candidate_1]) - math::eps))
            {
                n.e[candidate_0] = 0; n = n.normalized();
                //assert(n.z() != 0);
            }
            else if (abs(n.e[candidate_1]) < (abs(n.e[candidate_0]) - math::eps))
            {
                n.e[candidate_1] = 0; n = n.normalized();
                //assert(n.z() != 0);
            }
            else
            {
                // Resolve cells pointed to by each candidate
                math::vec<3> neighbour_0 = n;
                math::vec<3> neighbour_1 = n;
                neighbour_0.e[candidate_0] = 0;
                neighbour_1.e[candidate_1] = 0;
                math::vec<3> n0 = neighbour_0.normalized();
                math::vec<3> n1 = neighbour_1.normalized();
                neighbour_0 = n0 + uvw_i;
                neighbour_1 = n1 + uvw_i;
                if (neighbour_0.x() > geometry::vol::width || neighbour_0.x() < 0 ||
                    neighbour_0.y() > geometry::vol::width || neighbour_0.y() < 0 ||
                    neighbour_0.z() > geometry::vol::width || neighbour_0.z() < 0) // Can probably replace these tests with [geometry::vol::cell_validity(...)]
                {
                    // Zeroth neighbour lies outside the volume, zeroth candidate normal is safe to use
                    // Future versions will perform an occupancy test here
                    n = n0;
                }
                else if (neighbour_1.x() > geometry::vol::width || neighbour_1.x() < 0 ||
                         neighbour_1.y() > geometry::vol::width || neighbour_1.y() < 0 ||
                         neighbour_1.z() > geometry::vol::width || neighbour_1.z() < 0)
                {
                    // First neighbour lies outside the volume, first candidate normal is safe to use
                    // Future versions will perform an occupancy test here
                    n = n1;
                }
                else // Both neighbours lie inside an occupied voxel; invalid, suggests something wrong with our normal math
                     // Only hit this codepath in release mode - seems related to fp/fast and fp intrinsics compiler settings, no longer hit after turning those off
                {
                    DebugBreak();
                    n = cellOffs;
                    n.e[candidate_0] = 0;
                    n.e[candidate_1] = 0;
                    n = n.normalized();
                }
                //assert(n.z() != 0);
            }

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
            else // Rays get trapped inside surfaces very frequently in release, still not sure why
            {
                // Map outgoing direction back from sampling-space to worldspace
                math::m3 nSpace = math::normalSpace(n);
                out_vt.dir = nSpace.apply(out_vt.dir).normalized();

                // Push outgoing rays slightly out of the surface (to help reduce shadow acne)
                out_vt.ori = curr_ray.ori - (curr_ray.dir * 0.001f);
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

        // Update bounce counter (escaped rays still bounce exactly once off the sky)
        bounceCtr++;
    }
}
