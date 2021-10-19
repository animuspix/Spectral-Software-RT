export module scene;

import vmath;
import path;
import vox_ints;
import geometry;
import parallel;
import platform;
import materials;
import spectra;
import lights;

export namespace scene
{
    // Traverse the scene, pass path vertices back up to our pipeline so they can be integrated separately from scene traversal
    // (allowing for BDPT/VCM and other integration schemes besides regular unidirectional)
    void isect(tracing::path_vt init_vt, tracing::path* vertex_output, float* isosurf_dist, u32 tileNdx)
    {
        float horizon_dist = 1000.0f;
        typedef tracing::path_vt ray;
        ray curr_ray(init_vt);
        ray out_vt = curr_ray; // Output vertex for each bounce is always one iteration behind the current ray (since those are intended to be setting up the _next_ bounce in each path)
        bool path_absorbed = false;
        bool path_escaped = false;
        u8 bounceCtr = 0;
        geometry::vol::vol_nfo volume_nfo;
        bool within_grid = geometry::test(curr_ray.dir, &curr_ray.ori, &volume_nfo);
        //#define VALIDATE_VERTEX_COUNTS
        //#define PROPAGATION_DBG
        //#define VALIDATE_BOUNCES
//#define VALIDATE_BOUNDARY_SCATTERING
#ifdef VALIDATE_BOUNCES
        bool validating_path_bounces = false;
#endif
        vmath::vec<3> uvw_i_prev = vmath::vec<3>(9999.9f, 9999.9f, 9999.9f);
        //bool mask_current_cell = false; // Flag preventing self-intersection when we're bouncing out of volume cells
#ifdef VALIDATE_VERTEX_COUNTS
        u32 init_vt_count = vertex_output->front;
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
                vmath::vec<3> rel_p, uvw, uvw_scaled, uvw_i; // Forward-declare variables we'll need to resolve the voxel we've hit once we've stepped to the isosurface
                if (first_grid_hit && *isosurf_dist > -1.0f) // Isosurface distances initialize to [-1]; zero distances are reserved for subpixels that immediately touch a grid boundary
                {
                    curr_ray.ori += curr_ray.dir * *isosurf_dist;
                }

                // March from the current position to the first cell with non-zero state
                bool surf_found = false;
                bool ray_escaped = false;
                vmath::vec<3> grid_isect_pos = curr_ray.ori;
                vmath::vec<3> voxel_normal = vmath::vec<3>(0, 0, 1);
                while (!surf_found && !ray_escaped) // We want to resolve floored voxel coordinates at least once before looking anything up
                {
                    // Resolve intersection cell each tap
                    rel_p = (curr_ray.ori - volume_nfo.transf.pos) + (volume_nfo.transf.scale * 0.5f); // Relative position from lower object corner
                    uvw = rel_p / volume_nfo.transf.scale; // Normalized UVW
                    uvw_scaled = uvw * geometry::vol::width; // Voxel coordinates! :D
                    uvw_i = vmath::vfloor(uvw_scaled);

                    // Occasionally voxel coordinates can go negative right after intersection, due to floating-point error; make sure to handle that here
                    if (vmath::anyLesser(uvw_i, 0.0f))
                    {
                        uvw_i = vmath::vmax(uvw_i, vmath::vec<3>(0, 0, 0));
                        ray_escaped = true;
                        within_grid = false;
                        break;
                    }
                    //#define VALIDATE_TRAVERSAL_ISOLATED
#ifndef VALIDATE_TRAVERSAL_ISOLATED
                    else if (geometry::volume->test_cell_state(uvw_i) == geometry::vol::CELL_STATUS::OCCUPIED && !(vmath::allEqual(uvw_i, uvw_i_prev))) // Only allow this branch if we're currently traversing the grid (not bouncing),
                                                                                                                                                       // or if we're on the zeroth bounce and we've hit a boundary cell
#else // Optionally remove voxel state tests to check possible traversal issues in isolation
                    bool sdf_state_test = (vmath::vec<3>(512, 512, 512) - uvw_i).magnitude() < 512.0f; // Always use a sphere centered at the origin for debugging here
                    if (sdf_state_test && !mask_current_cell)
#endif
                    {
                        surf_found = true;
                        break;
                    }
                    else
                    {
                        // Find the next cell intersection, and step into it before recalculating rel_p/uvw & checking occupancy again
                        vmath::vec<3> input_ro = curr_ray.ori;
                        bool cell_step_success = geometry::test_cell_intersection(curr_ray.dir, &curr_ray.ori, uvw_i, &voxel_normal);
                        if (!cell_step_success) // The only way for this to happen is usually for a ray to be scattering at the boundary of the volume and refract out; if every neighbour fails
                                                // to intersect, something is probably wrong
                        {
#ifdef VALIDATE_BOUNDARY_SCATTERING
                            if (vmath::allGreater(uvw_i, 0.0f) && vmath::allLesser(uvw_i, static_cast<float>(geometry::vol::max_cell_ndx_per_axis)))
                            {
                                parallel::append_tile_log(tileNdx, "ray dropped within volume interior\n");
                                parallel::append_tile_log(tileNdx, "testing coordinates %f, %f, %f\n", uvw_i.x(), uvw_i.y(), uvw_i.z());
                                //platform::osDebugBreak();
                            }
#endif
                            uvw_i = vmath::vmax(uvw_i, vmath::vec<3>(0, 0, 0));
                            ray_escaped = true;
                            within_grid = false;
                            break;
                        }

                        //#define BLOCK_TRAVERSAL
#ifdef BLOCK_TRAVERSAL
                        ray_escaped = true;
                        within_grid = false;
                        break;
#endif
                        // Debug info for traversal steps
#ifdef PROPAGATION_DBG
                        vmath::vec<3> rel_p2 = (curr_ray.ori - volume_nfo.transf.pos) + (volume_nfo.transf.scale * 0.5f);
                        vmath::vec<3> uvw2 = rel_p / volume_nfo.transf.scale;
                        vmath::vec<3> uvw_scaled2 = uvw2 * geometry::vol::width;
                        vmath::vec<3> uvw_i2 = vmath::floor(uvw_scaled2);
                        if (vmath::anyGreater(uvw_i2, 1024.0f) || vmath::anyLesser(uvw_i2, 0.0f))
                        {
                            platform::osDebugBreak();
                        }
                        // Debug outputs for input/output ray positions
                        /*vmath::vec<3> d_ro = curr_ray.ori - input_ro;
                        strm << "input ray position (" << input_ro.x() << "," << input_ro.y() << "," << input_ro.z() << ")\n";
                        strm << "output ray position (" << curr_ray.ori.x() << "," << curr_ray.ori.y() << "," << curr_ray.ori.z() << ")\n";
                        strm << "delta (" << d_ro.x() << "," << d_ro.y() << "," << d_ro.z() << ")\n\n";

                        // Debug outputs for input/output ray positions relative to the lower-left corner of the volume
                        vmath::vec<3> rel_p2 = (curr_ray.ori - volume_nfo.transf.pos) + (volume_nfo.transf.scale * 0.5f);
                        vmath::vec<3> drel_p2 = rel_p2 - rel_p;
                        strm << "input relative ray pos (" << rel_p.x() << "," << rel_p.y() << "," << rel_p.z() << ")\n";
                        strm << "expected output relative ray pos (" << rel_p2.x() << "," << rel_p2.y() << "," << rel_p2.z() << ")\n";
                        strm << "delta (" << rel_p.x() << "," << rel_p.y() << "," << rel_p.z() << ")\n\n";

                        // Debug outputs for input/output voxel coordinates
                        vmath::vec<3> uvw2 = rel_p / volume_nfo.transf.scale;
                        vmath::vec<3> uvw_scaled2 = uvw2 * geometry::vol::width;
                        vmath::vec<3> uvw_i2 = vmath::floor(uvw_scaled2);
                        vmath::vec<3> duvw2 = uvw2 - uvw, duvw_scaled2 = uvw_scaled2 - uvw_scaled, duvw_i2 = uvw_i2 - uvw;
                        strm << "input voxel coordinates, normalized (" << uvw.x() << "," << uvw.y() << "," << uvw.z() << ")"
                             << ", scaled (" << uvw_scaled.x() << "," << uvw_scaled.y() << "," << uvw_scaled.z() << ")"
                             << ", floored/integral (" << uvw_i.x() << "," << uvw_i.y() << "," << uvw_i.z() << ")" << "\n";
                        strm << "expected output voxel coordinates, normalized (" << uvw2.x() << "," << uvw2.y() << "," << uvw2.z() << ")"
                             << ", scaled (" << uvw_scaled2.x() << "," << uvw_scaled2.y() << "," << uvw_scaled2.z() << ")"
                             << ", floored/integral (" << uvw_i2.x() << "," << uvw_i2.y() << "," << uvw_i2.z() << ")" << "\n";
                        strm << "delta, normalized (" << duvw2.x() << "," << duvw2.y() << "," << duvw2.z()
                             << ", scaled (" << duvw_scaled2.x() << "," << duvw_scaled2.y() << "," << duvw_scaled2.z() << ")"
                             << ", floored/integral (" << duvw_i2.x() << "," << duvw_i2.y() << "," << duvw_i2.z() << ")" << ")\n\n";
                        DBG_PRINT(strm);
                        strm.clear();*/
#endif
                    }
#ifdef _DEBUG
                    platform::osAssertion(uvw_i.x() >= 0.0f && uvw_i.y() >= 0.0f && uvw_i.z() >= 0.0f); // Voxel coordinates should never be negative
#endif
                }

                // Update isosurface distances on first grid intersections
                if (first_grid_hit) *isosurf_dist = (curr_ray.ori - grid_isect_pos).magnitude();

                // Skip remaining tracing work if we've left the grid boundaries
                if (within_grid)
                {
                    // Sample surfaces (just lambertian diffuse for now)
                    switch (volume_nfo.mat.material_type)
                    {
                    case material_labels::DIFFUSE:
                        float sample[4];
                        parallel::rand_streams[tileNdx].next(sample);
                        materials::diffuse_surface_sample(&out_vt.dir, &out_vt.pdf, sample[0], sample[1]);
                        materials::diffuse_lambert_reflection(out_vt.rho_sample, volume_nfo.mat.spectral_response, curr_ray.ori);
                        break;
                    default:
                        platform::osDebugBreak(); // Unsupported material ;_;
                        break;
                    }

                    // Not complex enough geometry for absorption to happen atm, process it anyways~
                    if (curr_ray.rho_weight <= vmath::eps)
                    {
                        path_absorbed = true;
#ifdef VALIDATE_BOUNCES
                        if (validating_path_bounces)
                        {
                            parallel::append_tile_log(tileNdx, "ray absorbed");
                        }
#endif
                    }
                    else // Rays get trapped inside surfaces very frequently in release, still not sure why
                    {
                        // Map outgoing direction back from sampling-space to worldspace
                        vmath::m3 nSpace = vmath::normalSpace(voxel_normal);
                        out_vt.dir = nSpace.apply(out_vt.dir).normalized();
                        out_vt.ori = curr_ray.ori;
                        out_vt.mat = &volume_nfo.mat;

                        // Cache path vertex for integration
                        vertex_output->push(out_vt);
#ifdef VALIDATE_BOUNCES
#ifdef VALIDATE_BOUNCE_WORLD_COORDS
                        parallel::append_tile_log(tileNdx, "bounce occurred at %f, %f, %f, \n", out_vt.ori.x(), out_vt.ori.y(), out_vt.ori.z())
#else
                        parallel::append_tile_log(tileNdx, "bounce occurred at %f, %f, %f, \n", uvw_i.x(), uvw_i.y(), uvw_i.z());
                        validating_path_bounces = true;
#endif
#endif
#ifdef VALIDATE_VERTEX_COUNTS
                        parallel::append_tile_log(tileNdx, "vertex count %i, initially %i, \n", vertex_output->front, init_vt_count);
#endif
                        // Update current ray
                        curr_ray = out_vt;

                        // Record the current voxel for the next bounce, to help prevent self-intersection
                        uvw_i_prev = uvw_i;
                    }
                }
            }
            else
            {
                // Map exiting rays onto the sky (set 1000 units from the scene origin)
                out_vt.ori = curr_ray.ori + (curr_ray.dir * lights::sky_dist);

                // Sample our sky model (super wip)
//#define DEBUG_RED_SKY
#ifdef DEBUG_RED_SKY
                out_vt.rho_sample = 0.8f;
                out_vt.rho_weight = 1.0f;
#else
                out_vt.rho_weight = spectra::sky(out_vt.rho_sample, out_vt.dir.e[1]);
                out_vt.rho_weight *= lights::sky_env(&out_vt.pdf);
#endif

                // Pass exiting rays into our path buffer
                vertex_output->push(out_vt);
                path_escaped = true;

#ifdef VALIDATE_BOUNCES
                if (validating_path_bounces)
                {
                    parallel::append_tile_log(tileNdx, "ray escaped\n");
                }
#endif
            }

            // Update bounce counter (escaped rays still bounce exactly once off the sky)
            bounceCtr++;
        }
    }
};