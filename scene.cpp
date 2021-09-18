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
void scene::isect(path::path_vt init_vt, path* vertex_output, float* isosurf_dist, uint32_t tileNdx)
{
    typedef path::path_vt ray;
    ray curr_ray(init_vt);
    ray out_vt = curr_ray; // Output vertex for each bounce is always one iteration behind the current ray (since those are intended to be setting up the _next_ bounce in each path)
    bool path_absorbed = false;
    bool path_escaped = false;
    uint8_t bounceCtr = 0;
    geometry::vol::vol_nfo volume_nfo;
    bool within_grid = geometry::test(curr_ray.dir, &curr_ray.ori, &volume_nfo);
//#define VALIDATE_VERTEX_COUNTS
#ifdef VALIDATE_VERTEX_COUNTS
    uint32_t init_vt_count = vertex_output->front;
#endif
    while (!path_absorbed && !path_escaped)
    {
        // Test for intersection
        bool first_grid_hit = (bounceCtr == 0 && within_grid); // If we've just hit the volume grid, set this so we can use stored distances to avoid marching towards isoboundaries
        if (within_grid) // Within the grid, march until we leave the bounding box
        {
            // March/jump to intersections within the voxel grid
            /////////////////////////////////////////////////////////

            // Special case for silhouettes; if we've just hit the grid boundaries, jump to the surface instead of recalculating that distance every sample
            math::vec<3> rel_p, uvw, uvw_scaled, uvw_i; // Forward-declare variables we'll need to resolve the voxel we've hit once we've stepped to the isosurface
            if (first_grid_hit && *isosurf_dist > -1.0f) // Isosurface distances initialize to [-1]; zero distances are reserved for subpixels that immediately touch a grid boundary
            {
                curr_ray.ori += curr_ray.dir * *isosurf_dist;
            }

            // March from the current position to the first cell with non-zero state
            bool surf_found = false;
            bool ray_escaped = false;
            math::vec<3> grid_isect_pos = curr_ray.ori;
            while (!surf_found && !ray_escaped) // We want to resolve floored voxel coordinates at least once before looking anything up
            {
                // Resolve intersection cell each tap
                rel_p = (curr_ray.ori - volume_nfo.transf.pos) + (volume_nfo.transf.scale * 0.5f); // Relative position from lower object corner
                uvw = rel_p / volume_nfo.transf.scale; // Normalized UVW
                uvw_scaled = uvw * geometry::vol::width; // Voxel coordinates! :D
                uvw_i = math::floor(uvw_scaled);
                if (geometry::volume->test_cell_state(uvw_i) == geometry::vol::CELL_STATUS::OCCUPIED)
                {
                    surf_found = true;
                    break;
                }
                else
                {
                    // Find the next cell intersection, and step into it before recalculating rel_p/uvw & checking occupancy again
                    math::vec<3> input_ro = curr_ray.ori;
                    bool cell_step_success = geometry::test_cell_intersection(curr_ray.dir, &curr_ray.ori, uvw_i);
                    if (!cell_step_success) // The only way for this to happen is usually for a ray to be scattering at the boundary of the volume and refract out; if every neighbour fails
                                            // to intersect, something is probably wrong
                    {
                        uvw_i = math::max(uvw_i, math::vec<3>(0, 0, 0));
                        ray_escaped = true;
                        within_grid = false;
                        break;
                    }

//#define PROPAGATION_DBG
#ifdef PROPAGATION_DBG
                    // Debug outputs for input/output ray positions
                    std::stringstream str_strm;
                    math::vec<3> d_ro = curr_ray.ori - input_ro;
                    str_strm << "input ray position (" << input_ro.x() << "," << input_ro.y() << "," << input_ro.z() << ")\n";
                    str_strm << "output ray position (" << curr_ray.ori.x() << "," << curr_ray.ori.y() << "," << curr_ray.ori.z() << ")\n";
                    str_strm << "delta (" << d_ro.x() << "," << d_ro.y() << "," << d_ro.z() << ")\n\n";

                    // Debug outputs for input/output ray positions relative to the lower-left corner of the volume
                    math::vec<3> rel_p2 = (curr_ray.ori - volume_nfo.transf.pos) + (volume_nfo.transf.scale * 0.5f);
                    math::vec<3> drel_p2 = rel_p2 - rel_p;
                    str_strm << "input relative ray pos (" << rel_p.x() << "," << rel_p.y() << "," << rel_p.z() << ")\n";
                    str_strm << "expected output relative ray pos (" << rel_p2.x() << "," << rel_p2.y() << "," << rel_p2.z() << ")\n";
                    str_strm << "delta (" << rel_p.x() << "," << rel_p.y() << "," << rel_p.z() << ")\n\n";

                    // Debug outputs for input/output voxel coordinates
                    math::vec<3> uvw2 = rel_p / volume_nfo.transf.scale;
                    math::vec<3> uvw_scaled2 = uvw2 * geometry::vol::width;
                    math::vec<3> uvw_i2 = math::floor(uvw_scaled2);
                    math::vec<3> duvw2 = uvw2 - uvw, duvw_scaled2 = uvw_scaled2 - uvw_scaled, duvw_i2 = uvw_i2 - uvw;
                    str_strm << "input voxel coordinates, normalized (" << uvw.x() << "," << uvw.y() << "," << uvw.z() << ")"
                             << ", scaled (" << uvw_scaled.x() << "," << uvw_scaled.y() << "," << uvw_scaled.z() << ")"
                             << ", floored/integral (" << uvw_i.x() << "," << uvw_i.y() << "," << uvw_i.z() << ")" << "\n";
                    str_strm << "expected output voxel coordinates, normalized (" << uvw2.x() << "," << uvw2.y() << "," << uvw2.z() << ")"
                             << ", scaled (" << uvw_scaled2.x() << "," << uvw_scaled2.y() << "," << uvw_scaled2.z() << ")"
                             << ", floored/integral (" << uvw_i2.x() << "," << uvw_i2.y() << "," << uvw_i2.z() << ")" << "\n";
                    str_strm << "delta, normalized (" << duvw2.x() << "," << duvw2.y() << "," << duvw2.z()
                             << ", scaled (" << duvw_scaled2.x() << "," << duvw_scaled2.y() << "," << duvw_scaled2.z() << ")"
                             << ", floored/integral (" << duvw_i2.x() << "," << duvw_i2.y() << "," << duvw_i2.z() << ")" << ")\n\n";
                    OutputDebugStringA(str_strm.str().c_str());
#endif
                }
#ifdef _DEBUG
                assert(uvw_i.x() >= 0.0f && uvw_i.y() >= 0.0f && uvw_i.z() >= 0.0f); // Voxel coordinates should never be negative
#endif
            }

            // Update isosurface distances on first grid intersections
            if (first_grid_hit) *isosurf_dist = (curr_ray.ori - grid_isect_pos).magnitude();

            // Skip remaining tracing work if we've left the grid boundaries
            if (within_grid)
            {
                // Compute normal at the final intersection site
                // Special analytical normal solver instead of an approximation because we know each intersection will hit another cubey voxel; the uncertainty is which face we
                // hit, and not the overall curvature at the hit site
                /////////////////////////////////////////////////////

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
                    if ((neighbour_0.x() > geometry::vol::width || neighbour_0.x() < 0 || // Test if the zeroth neighbour lies completely outside the voxel grid
                         neighbour_0.y() > geometry::vol::width || neighbour_0.y() < 0 ||
                         neighbour_0.z() > geometry::vol::width || neighbour_0.z() < 0) ||
                        (geometry::volume->test_cell_state(neighbour_0) == geometry::vol::CELL_STATUS::EMPTY)) // If not, test if the cell it points into is occupied
                    {
                        // Zeroth neighbour lies outside the volume, zeroth candidate normal is safe to use
                        n = n0;
                    }
                    else if ((neighbour_1.x() > geometry::vol::width || neighbour_1.x() < 0 || // Test if the first neighbour lies completely outside the voxel grid
                              neighbour_1.y() > geometry::vol::width || neighbour_1.y() < 0 ||
                              neighbour_1.z() > geometry::vol::width || neighbour_1.z() < 0) ||
                             (geometry::volume->test_cell_state(neighbour_1) == geometry::vol::CELL_STATUS::EMPTY))
                    {
                        // First neighbour lies outside the volume, first candidate normal is safe to use
                        n = n1;
                    }
                    else // Both neighbours lie inside an occupied voxel; invalid, suggests something wrong with our normal math
                         // Only hit this codepath in release mode - seems related to fp/fast and fp intrinsics compiler settings, no longer hit after turning those off
                    {
                        std::stringstream strm_cell_dbg;
                        strm_cell_dbg << "source cell is " << ((geometry::volume->test_cell_state(uvw_i) == geometry::vol::CELL_STATUS::EMPTY) ? "empty" : "occupied") << '\n';
                        strm_cell_dbg << "left x-neighbour is " << ((geometry::volume->test_cell_state(uvw_i - math::vec<3>(1, 0, 0)) == geometry::vol::CELL_STATUS::EMPTY) ? "empty" : "occupied") << '\n';
                        strm_cell_dbg << "right x-neighbour is " << ((geometry::volume->test_cell_state(uvw_i + math::vec<3>(1, 0, 0)) == geometry::vol::CELL_STATUS::EMPTY) ? "empty" : "occupied") << '\n';
                        strm_cell_dbg << "top y-neighbour is " << ((geometry::volume->test_cell_state(uvw_i + math::vec<3>(0, 1, 0)) == geometry::vol::CELL_STATUS::EMPTY) ? "empty" : "occupied") << '\n';
                        strm_cell_dbg << "bottom y-neighbour is " << ((geometry::volume->test_cell_state(uvw_i - math::vec<3>(0, 1, 0)) == geometry::vol::CELL_STATUS::EMPTY) ? "empty" : "occupied") << '\n';
                        strm_cell_dbg << "forward z-neighbour is " << ((geometry::volume->test_cell_state(uvw_i + math::vec<3>(0, 0, 1)) == geometry::vol::CELL_STATUS::EMPTY) ? "empty" : "occupied") << '\n';
                        strm_cell_dbg << "backward z-neighbour is " << ((geometry::volume->test_cell_state(uvw_i - math::vec<3>(0, 0, 1)) == geometry::vol::CELL_STATUS::EMPTY) ? "empty" : "occupied") << '\n';
                        OutputDebugStringA(strm_cell_dbg.str().c_str());
                        DebugBreak();
                        n = cellOffs;
                        n.e[candidate_0] = 0;
                        n.e[candidate_1] = 0;
                        n = n.normalized();
                    }
                    //assert(n.z() != 0);
                }

                // Sample surfaces (just lambertian diffuse for now)
                switch (volume_nfo.mat.material_type)
                {
                    case material_labels::DIFFUSE:
                        float sample[4];
                        sampler::rand_streams[tileNdx].next(sample);
                        materials::diffuse_surface_sample(&out_vt.dir, &out_vt.pdf, sample[0], sample[1]);
                        materials::diffuse_lambert_reflection(out_vt.rho_sample, volume_nfo.mat.spectral_response, curr_ray.ori);
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
                    out_vt.mat = &volume_nfo.mat;

                    // Cache path vertex for integration
                    vertex_output->push(out_vt);
#ifdef VALIDATE_VERTEX_COUNTS
                    std::stringstream strm;
                    strm << "vertex count " << vertex_output->front << ", initially " << init_vt_count << "\n";
                    OutputDebugStringA(strm.str().c_str());
#endif
                    // Update current ray
                    curr_ray = out_vt;
                    within_grid = false;
                }
            }
        }
        else
        {
            // Map exiting rays onto the sky (set 1000 units from the scene origin)
            out_vt.ori = curr_ray.ori + (curr_ray.dir * lights::sky_dist);

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
