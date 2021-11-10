export module geometry;

#pragma once

import vmath;
import materials;
import mem;
import platform;
import parallel;
import spectra;
import vox_ints;

//#define GEOMETRY_DBG
#ifdef GEOMETRY_DBG
#pragma optimize("", off)
#endif

namespace geometry
{
    export struct vol
    {
        static constexpr u32 width = 1024; // All volumes are 1024 * 1024 * 1024
        static constexpr u32 slice_area = width * width;
        static constexpr u32 res = slice_area * width;
        static constexpr u32 chunk_res_x = 4;
        static constexpr u32 chunk_res_y = 4;
        static constexpr u32 chunk_res_z = 4;
        static constexpr u32 num_chunks_x = width / chunk_res_x;
        static constexpr u32 num_chunks_y = width / chunk_res_y;
        static constexpr u32 num_chunks_z = width / chunk_res_z;
        static constexpr u32 num_chunks_xy = num_chunks_x * num_chunks_y;
        static constexpr u32 num_chunks = res / (chunk_res_x * chunk_res_y * chunk_res_z);
        static constexpr float cell_size = 1.0f / width;
        static constexpr u32 max_cell_ndx_per_axis = width - 1;
        struct transform_nfo
        {
            vmath::vec<3> scale; // Bounding-box scale on x/y/z
            vmath::vec<3> pos; // World-space position
            vmath::vec<4> orientation; // World-space orientation, specified as a quaternion
            vmath::vec<2> ss_v0; // Top-left of the screen-space quad projected from the volume's bounding-box, updated on camera transform
            vmath::vec<2> ss_v1; // Top-right of the screen-space bounding quad
            vmath::vec<2> ss_v2; // Bottom-left of the screen-space bounding quad
            vmath::vec<2> ss_v3; // Bottom-right of the screen-space bounding quad
        };

        // Volume metadata (material, transform information)
        struct vol_nfo
        {
            materials::instance mat;
            transform_nfo transf;
        };
        vol_nfo metadata;
        u64 cell_states[num_chunks]; // One bit/cell; order is left-right/front-back/top-bottom
                                     // front view:
                                     // 00 01 02 03
                                     // 16 17 18 19
                                     // 32 33 34 35
                                     // 48 49 50 51
                                     // top view:
                                     // 00 01 02 03
                                     // 04 05 06 07
                                     // 08 09 10 11
                                     // 12 13 14 15
                                     // bottom view:
                                     // 48 49 50 51
                                     // 52 53 54 55
                                     // 56 57 58 59
                                     // 60 61 62 63
                                     // To test:
                                     // - Evaluate cell & 0b00000000; if zero, step two cells
                                     // - If not zero, test the bit at the current intersection and branch (either step one cell forward, or evaluate the current material)
        static constexpr u32 footprint = res + sizeof(vol_nfo);
        static u32 index_solver(vmath::vec<3> uvw_floored, u64* out_bit_ndx_mask) // Returns chunk index directly, bitmask to select the cell within the chunk in [out_bit_ndx_mask]
        {
            u8 bit_ndx = static_cast<u8>((floor(fmodf(uvw_floored.z(), 4.0f)) * 4) + // Z-axis, snapped into chunk space
                                         (floor(fmodf(uvw_floored.y(), 4.0f)) * 16) + // Y-axis, snapped into chunk space
                                         floor(fmodf(uvw_floored.x(), 4.0f))); // X-axis, snapped into chunk space
            *out_bit_ndx_mask = 1ull << bit_ndx;
            uvw_floored = vmath::vfloor(uvw_floored / 4.0f);
            return static_cast<u32>(uvw_floored.x() + // Local scanline offset
                                   (uvw_floored.y() * num_chunks_x) + // Local slice offset
                                   (uvw_floored.z() * num_chunks_xy)); // Volume offset;
        }
        static vmath::vec<3> uvw_solver(u32 cell_ndx) // Designed for mapping [0...res] loop iterations into positions to simplify vmaths operations; not designed to work with bitfields
                                                      // and chunk indices as input
        {
            u32 single_page_ndx = cell_ndx % slice_area;
            return vmath::vec<3>(static_cast<float>(single_page_ndx % width),
                static_cast<float>(single_page_ndx / width),
                static_cast<float>(cell_ndx / slice_area));
        }
        static vmath::vec<3> uvw_solver_chunked(u32 chunk_ndx)
        {
            const u32 chunked_area = num_chunks_x * num_chunks_y;
            const u32 chunked_width = num_chunks_x;
            u32 single_page_ndx = chunk_ndx % chunked_area;
            return vmath::vec<3>(static_cast<float>(single_page_ndx % chunked_width),
                static_cast<float>(single_page_ndx / chunked_width),
                static_cast<float>(chunk_ndx / chunked_area));
        }

        // Helper enum & functions to quickly parse chunk/cell data for intersection tests + cell updates
        enum class CELL_STATUS
        {
            OCCUPIED,
            EMPTY
        };
        void set_cell_state(CELL_STATUS status, vmath::vec<3> uvw_floored)
        {
            u64 bit_selector = 0x0;
            u32 chunk_ndx = index_solver(uvw_floored, &bit_selector);
            u64& s = cell_states[chunk_ndx];
            if (status == CELL_STATUS::OCCUPIED) s |= bit_selector;
            else { s &= ~bit_selector; }
        }
        void set_chunk_state(u64 state, u32 chunk_ndx)
        {
            cell_states[chunk_ndx] = state;
        }
        CELL_STATUS test_cell_state(vmath::vec<3> uvw_floored)
        {
            u64 bit_selector = 0x0;
            u32 chunk_ndx = index_solver(uvw_floored, &bit_selector);
            u64 s = cell_states[chunk_ndx];
            if (s > 0)
            {
                return (s & bit_selector) > 0 ? CELL_STATUS::OCCUPIED :
                                                CELL_STATUS::EMPTY;
            }
            else
            {
                // Zero state, no voxels are set, return immediately
                return CELL_STATUS::EMPTY;
            }
        }

        // Just coarser-grained [test_cell_state], possibly useful for quickly skipping throuhg geometry
        CELL_STATUS test_chunk_state(vmath::vec<3> uvw_floored)
        {
            u64 bit_selector = 0x0;
            u32 chunk_ndx = index_solver(uvw_floored, &bit_selector);
            u64 s = cell_states[chunk_ndx];
            if (s > 0)
            {
                return CELL_STATUS::OCCUPIED;
            }
            else
            {
                // Zero state, no voxels are set, return immediately
                return CELL_STATUS::EMPTY;
            }
        }

        // Resolve screen-space volume bounds for the current camera transform
        // (inverse lens sampling performed on the camera, so this just needs to resolve worldspace bounds and reproject them that way, before
        // taking the min/max coordinates in the 2D plane and storing those)
        void resolveSSBounds(vmath::vec<2>(*inverse_lens_sampler_fn)(vmath::vec<3>))
        {
            // Resolve worldspace extents for our volume AABB
            // Will eventually need to adjust this code for different camera angles, zoom, panning, etc.
            const vmath::vec<3> vol_p = metadata.transf.pos;
            const vmath::vec<3> vol_extents = metadata.transf.scale * 0.5f;

            // Volume AABB vertices, from left->right, top->bottom, and front->back
            ///////////////////////////////////////////////////////////////////////

            // Front
            const vmath::vec<3> aabb0 = vol_p + vmath::vec<3>(-vol_extents.x(), vol_extents.y(), -vol_extents.z());
            const vmath::vec<3> aabb1 = vol_p + vmath::vec<3>(vol_extents.x(), vol_extents.y(), -vol_extents.z());
            const vmath::vec<3> aabb2 = vol_p + vmath::vec<3>(-vol_extents.x(), -vol_extents.y(), -vol_extents.z());
            const vmath::vec<3> aabb3 = vol_p + vmath::vec<3>(vol_extents.x(), -vol_extents.y(), -vol_extents.z());

            // Back
            const vmath::vec<3> aabb4 = vol_p + vmath::vec<3>(-vol_extents.x(), vol_extents.y(), vol_extents.z());
            const vmath::vec<3> aabb5 = vol_p + vmath::vec<3>(vol_extents.x(), vol_extents.y(), vol_extents.z());
            const vmath::vec<3> aabb6 = vol_p + vmath::vec<3>(-vol_extents.x(), -vol_extents.y(), vol_extents.z());
            const vmath::vec<3> aabb7 = vol_p + vmath::vec<3>(vol_extents.x(), -vol_extents.y(), vol_extents.z());

            // Project AABB vertices into screenspace
            const vmath::vec<2> aabb_vertices_ss[8] = { inverse_lens_sampler_fn(aabb0),
                                                        inverse_lens_sampler_fn(aabb1),
                                                        inverse_lens_sampler_fn(aabb2),
                                                        inverse_lens_sampler_fn(aabb3),
                                                        inverse_lens_sampler_fn(aabb4),
                                                        inverse_lens_sampler_fn(aabb5),
                                                        inverse_lens_sampler_fn(aabb6),
                                                        inverse_lens_sampler_fn(aabb7) };

            // Resolve screen-space quad from min/max vertices
            vmath::vec<4> min_max_px = vmath::vec<4>(9999.9f, 9999.9f, -9999.9f, -9999.9f); // Order is minX, minY, maxX, maxY
            for (u8 i = 0; i < 8; i++)
            {
                float vx = aabb_vertices_ss[i].e[0], vy = aabb_vertices_ss[i].e[1];
                min_max_px.e[0] = vmath::fmin(min_max_px.e[0], vx);
                min_max_px.e[1] = vmath::fmin(min_max_px.e[1], vy);
                min_max_px.e[2] = vmath::fmax(min_max_px.e[2], vx);
                min_max_px.e[3] = vmath::fmax(min_max_px.e[3], vy);
            }
            metadata.transf.ss_v0 = min_max_px.xy(); // Min, min
            metadata.transf.ss_v1 = vmath::vec<2>(min_max_px.x(), min_max_px.w()); // Min, max
            metadata.transf.ss_v2 = vmath::vec<2>(min_max_px.z(), min_max_px.y()); // Max, min
            metadata.transf.ss_v3 = min_max_px.zw(); // Max, max
        }
    };

    // Core volume info :D
    export vol* volume;

    // Volume initializer, either loads voxels from disk or generates them procedurally on startup
    void geom_setup(u16 num_tiles_x, u16 num_tiles_y, u16 tile_ndx)
    {
        const u32 num_tiles = static_cast<u32>(num_tiles_x) * num_tiles_y;
        const u32 slice_width = vol::num_chunks_z / num_tiles; // Width for most slices except the last one, good enough for offset maths
        const u32 init_z = static_cast<u32>(tile_ndx) * slice_width;
        const u32 max_z = vol::num_chunks_z - init_z;
        constexpr float r2 = 512.0f * 512.0f;
        constexpr float r2_shell_outer = (512.0f + (vol::chunk_res_x * 2.0f)) *
                                         (512.0f + (vol::chunk_res_x * 2.0f));
        constexpr float r2_shell_inner = (512.0f - (vol::chunk_res_x * 2.0f)) *
                                         (512.0f - (vol::chunk_res_x * 2.0f));
        const vmath::vec<3> circOrigin(512, 512, 512);
        u32 chunk_ctr = init_z * vol::num_chunks_xy;
        for (u32 i = init_z; i < max_z; i += 1)
        {
            for (u32 j = 0; j < vol::num_chunks_y; j += 1)
            {
                for (u32 k = 0; k < vol::num_chunks_x; k += 1)
                {
                    vmath::vec<3> chunk_uvw = vmath::vec<3>((float)k, (float)j, (float)i) *
                                              vmath::vec<3>(vol::chunk_res_x,
                                                            vol::chunk_res_y,
                                                            vol::chunk_res_z); // Scale up coordinates to meet the corners of each chunk in the volume grid
                    vmath::vec<3> disp_from_ori = circOrigin - chunk_uvw;
                    u32 x, y, z;
                    u64 chunk = 0;
                    const float conservative_dist = disp_from_ori.sqr_magnitude();
                    if (conservative_dist < r2_shell_outer) // Conservatively check if any cells in the chunk could be nonzero before testing them
                                                            // Future versions will encode empty runs into the files storing a volume and compare against those
                                                            // runs when they decide whether to process a chunk
                    {
                        // Huge thanks to nightchild on GP discord for the inspiration :D
                        if (conservative_dist > r2_shell_inner)
                        {
                            for (u32 cell = 0; cell < 64; cell++)
                            {
                                // front view:
                                // 00 01 02 03
                                // 16 17 18 19
                                // 32 33 34 35
                                // 48 49 50 51
                                // top view:
                                // 00 01 02 03
                                // 04 05 06 07
                                // 08 09 10 11
                                // 12 13 14 15
                                // bottom view:
                                // 48 49 50 51
                                // 52 53 54 55
                                // 56 57 58 59
                                // 60 61 62 63
                                x = cell % vol::chunk_res_x;
                                y = cell / (vol::chunk_res_x * vol::chunk_res_y);
                                z = cell / vol::chunk_res_x;
                                if ((disp_from_ori - vmath::vec<3>((float)x, (float)y, (float)z)).sqr_magnitude() < r2) chunk |= 1ull << cell;
                            }
                        }
                        else
                        {
                            // Chunks within the innner shell are definitely completely filled, so flush them here and avoid the loop above
                            chunk = 0xffffffffffffffff;
                        }
                    }
                    volume->set_chunk_state(chunk, chunk_ctr);
                    chunk_ctr++;
                }
            }
        };
    }

    export void init(vmath::vec<2>(*inverse_lens_sampler_fn)(vmath::vec<3>))
    {
        // Allocate volume memory
        volume = mem::allocate_tracing<vol>(vol::footprint); // Generalized volume info

        // Load/generate geometry
//#define TIMED_GEOMETRY_UPLOAD
#ifdef TIMED_GEOMETRY_UPLOAD
        double geom_setup_t = platform::osGetCurrentTimeSeconds();
#endif
        parallel::launch(geom_setup);

        // Wait for loader threads to finish
        bool loaded = false;
        while (!loaded)
        {
            bool loadTest = true;
            for (u32 i = 0; i < parallel::numTiles; i++)
            {
                loadTest = loadTest && parallel::tiles[i].state->load() == platform::threads::SLEEPING;
            }
            loaded = loadTest;
            if (loaded) break;
        }
#ifdef TIMED_GEOMETRY_UPLOAD
        platform::osDebugLogFmt("geometry loaded within %f seconds \n", platform::osGetCurrentTimeSeconds() - geom_setup_t);
        platform::osDebugBreak();
#endif
        // Possible debugging helper for geometry here; build in a .png exporter, write out cells on a certain slice to black or white depending on activation status
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Update transform metadata; eventually this should be loaded from disk
        // (position should probably be actually zeroed, there's no reason for users to modify it instead of moving the camera)
        volume->metadata.transf.pos = vmath::vec<3>(0.0f, 0.0f, 20.0f);
        volume->metadata.transf.orientation = vmath::vec<4>(0.0f, 0.0f, 0.0f, 1.0f);
        volume->metadata.transf.scale = vmath::vec<3>(4, 4, 4); // Boring regular unit scale
        volume->resolveSSBounds(inverse_lens_sampler_fn);

        // Update material metadata; eventually this should be loaded from disk
        materials::instance& boxMat = volume->metadata.mat;
        boxMat.material_type = material_labels::DIFFUSE;
        boxMat.roughness = 0.2f;
        boxMat.spectral_ior = vmath::fn<4, const float>(spectra::placeholder_spd);
        boxMat.spectral_response = vmath::fn<4, const float>(spectra::placeholder_spd);
    }

    // Test the bounding geometry for the volume grid
    // Used to quickly mask out rays that immediately hit the sky or an external light source
    export bool test(vmath::vec<3> dir, vmath::vec<3>* ro_inout, geometry::vol::vol_nfo* vol_nfo_out)
    {
        // Import object properties
        const vmath::vec<3> extents = volume->metadata.transf.scale * 0.5f; // Extents from object origin
        const vmath::vec<3> pos = volume->metadata.transf.pos;
        const vmath::vec<4> orientation = volume->metadata.transf.orientation;

        // Intersection vmath adapted from
        // https://www.shadertoy.com/view/ltKyzm, itself adapted from the Scratchapixel tutorial here:
        // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection

        // Synthesize box boundaries
        const vmath::vec<3> boundsMin = (pos - extents);
        const vmath::vec<3> boundsMax = (pos + extents);

        // Evaluate per-axis distances to each plane in the box
        // Not super sure why these divisions happen?
        // Should probably reread vmath for all this in general
        vmath::vec<3> plane_dists[2] =
        {
            (boundsMin - *ro_inout) / dir,
            (boundsMax - *ro_inout) / dir
        };

        // Keep near distances in [0], far distances in [1]
        const vmath::vec<3> dist0 = plane_dists[0];
        const vmath::vec<3> dist1 = plane_dists[1];
        plane_dists[0].e[0] = vmath::fmin(dist0.x(), dist1.x());
        plane_dists[0].e[1] = vmath::fmin(dist0.y(), dist1.y());
        plane_dists[0].e[2] = vmath::fmin(dist0.z(), dist1.z());
        plane_dists[1].e[0] = vmath::fmax(dist0.x(), dist1.x());
        plane_dists[1].e[1] = vmath::fmax(dist0.y(), dist1.y());
        plane_dists[1].e[2] = vmath::fmax(dist0.z(), dist1.z());

        // Evaluate scalar min/max distances for the given ray
        vmath::vec<2> sT = vmath::vec<2>(vmath::fmax(vmath::fmax(plane_dists[0].x(), plane_dists[0].y()), plane_dists[0].z()),
                                         vmath::fmin(vmath::fmin(plane_dists[1].x(), plane_dists[1].y()), plane_dists[1].z()));
        sT = vmath::vec<2>(vmath::fmin(sT.x(), sT.y()), vmath::fmax(sT.x(), sT.y())); // Keep near distance in [x], far distance in [y]

        // Resolve intersection status
        const bool isect = (plane_dists[0].x() < plane_dists[1].y() && plane_dists[0].y() < plane_dists[1].x() &&
                            plane_dists[0].z() < sT.y() && sT.x() < plane_dists[1].z()) && (sT.e[0] > 0); // Extend intersection test to ignore intersections behind the current ray  (where the direction
                                                                                                          // to the intersection point is the reverse of the current ray direction)

        // Write out intersection position + shared object info for successful intersections, then early-out
        // (need to sort candidates by depth here)
        if (isect)
        {
            *ro_inout = *ro_inout + (dir * sT.e[0]);
            *vol_nfo_out = volume->metadata;
            return true;
        }
        return false;
    }

    // Test for intersections with individual cells within the grid, using DDA
    // (single-cell steps within the volume grid, along a given direction)
    // Implemented following the guide in this tutorial
    // https://www.y()outube.com/watch?v=W5P8GlaEOSI
    // + this paper/blog
    // https://castingrays.blogspot.com/2014/01/voxel-rendering-using-discrete-ray.html
    // many thanks to the creators of both <3
    export bool cell_step(vmath::vec<3> dir, vmath::vec<3>* ro_inout, vmath::vec<3> uvw_in, vmath::vec<3>* uvw_i_inout, vmath::vec<3>* n_out, bool primary_ray)
    {
        // Safety test!
        // Make sure any rays that enter this function have safe starting values
        // Also validate against error cases (incoming coordinates more than one unit from a boundary)
        vmath::vec<3> uvw_floored = *uvw_i_inout;
        const vmath::vec<3> bounds_dist_max = uvw_in - vmath::vec<3>(vol::width, vol::width, vol::width);
        const vmath::vec<3> bounds_dist_min = vmath::vec<3>(0.0f, 0.0f, 0.0f) - uvw_floored;
        if (vmath::anyGreater(bounds_dist_max, 0.0f) || vmath::anyGreater(bounds_dist_min, -1.0f)) // coordinates equal to vol::width are still out of bounds :p
        {
            // Something funky, bad coordinates or generating coordinates when we're outside the scene bounding box
//#define VALIDATE_CELL_RANGES
#ifdef VALIDATE_CELL_RANGES
            if (vmath::anyGreater(bounds_dist_max, 1.0f) || vmath::anyGreater(bounds_dist_min, -1.0f))
            {
                platform::osDebugBreak();
            }
#endif

            // Most cases will be regular rounding error :D
            vmath::clamp(uvw_in, vmath::vec<3>(0.0f), vmath::vec<3>(vol::width-1));
            vmath::clamp(uvw_floored, vmath::vec<3>(0.0f), vmath::vec<3>(vol::width-1));
        }

        // We only traverse primary rays with an empty starting cell (since otherwise we can return immediately)
        // For non-primary rays (bounce rays) we ignore the starting cell and iterate through any others along the ray
        // direction
        bool traversing = primary_ray ? volume->test_cell_state(*uvw_i_inout) == vol::CELL_STATUS::EMPTY : true;

        // Core traversal loop/immediate return
        if (traversing)
        {
            // Compute change in uvw per-axis
            const vmath::vec<3> d_uvw = vmath::vsgn(dir);

            // Compute x/y/z-derivatives
            const vmath::vec<3> safe_dir_axes = vmath::vmax(vmath::vabs(dir), vmath::vec<3>(0.01f)) * d_uvw;
            const vmath::vec<3> g = vmath::vec<3>((dir / safe_dir_axes.e[0]).magnitude(), // Scalarized directional derivative relative to x
                                                  (dir / safe_dir_axes.e[1]).magnitude(), // Scalarized directional derivative relative to y
                                                  (dir / safe_dir_axes.e[2]).magnitude()); // Scalarized directional derivative realtive to z

            // Distances to the first cell interval/boundary
            vmath::vec<3> t = vmath::vec<3>((dir.e[0] >= 0 ? (uvw_floored.e[0] + 1.0f - uvw_in.e[0]) : uvw_in.e[0] - uvw_floored.e[0]) * g.e[0],
                                            (dir.e[1] >= 0 ? (uvw_floored.e[1] + 1.0f - uvw_in.e[1]) : uvw_in.e[1] - uvw_floored.e[1]) * g.e[1],
                                            (dir.e[2] >= 0 ? (uvw_floored.e[2] + 1.0f - uvw_in.e[2]) : uvw_in.e[2] - uvw_floored.e[2]) * g.e[2]);

            // DDA
            u8 min_axis = 0; // Smallest axis in our traversal vector, used to determine which direction to step through in each tap
            bool cell_found = false;
            while (!cell_found)
            {
                // Eventually should run a coarse test per-chunk here, followed by a fine test for nonzero chunks
                // (doable by modifying the step size & switching between testing at the chunk level vs testing at the voxel level,
                // much cheaper than nested loops)
                /////////////////////////////////////////////////////////////////////////////////////////////////

                // Minimize divergence from the ideal path through [dir] by always incrementing our smallest axis
                min_axis = t.x() < t.y() && t.x() < t.z() ? 0 :
                           t.y() < t.x() && t.y() < t.z() ? 1 :
                           /* d_pos.x() < d_pos.y() || d_pos.x() <= d_pos.z() */ 2 /* : 0*/;

                // We want to weight each continuous step by its axis' contribution to the slope of the ray direction
                t.e[min_axis] += g.e[min_axis]; // Optional scale here for chunk traversal

                // Update our current voxel coordinate
                uvw_floored.e[min_axis] += d_uvw.e[min_axis]; // Optional scale here for chunk traversal

                // Ray escaped the volume :o
                if (vmath::anyGreater(uvw_floored, vol::width - 1) || vmath::anyLesser(uvw_floored, 0))
                {
                    uvw_floored = vmath::clamp(uvw_floored, vmath::vec<3>(0, 0, 0), vmath::vec<3>(vol::width-1, vol::width-1, vol::width-1));
                    cell_found = false;
                    break;
                }

                // Successful intersection :D
                if (volume->test_cell_state(uvw_floored) == vol::CELL_STATUS::OCCUPIED)
                {
                    cell_found = true;
                    break;
                }
            }

            // Outputs :D
            // Only need to write these if we've traversed the grid, since they'll be the same as our inputs
            // otherwise
            ////////////////////////////////////////////////////////////////////////////////////////////////

            // Derive normal from most recent step (thanks nightchild from GP!)
            *n_out = min_axis == 0 ? vmath::vec<3>(-d_uvw.e[0], 0.0f, 0.0f) :
                     min_axis == 1 ? vmath::vec<3>(0.0f, -d_uvw.e[1], 0.0f) :
                     /*min_axis == 2 ? */vmath::vec<3>(0.0f, 0.0f, -d_uvw.e[2])/* : vmath::vec<3>(0, 0, -1.0f)*/;

            // Output integer UVW coordinate
            // We've already clamped it if we needed to, so a direct copy here is fine
            *uvw_i_inout = uvw_floored;

            // Calculate worldspace position delta, then update output coordinate
            /////////////////////////////////////////////////////////////////////

            // Reverse of the math we did in [scene.ixx] to resolve intersected voxel coordinates
            /////////////////////////////////////////////////////////////////////////////////////

            // Original math from [scene.ixx]
            //rel_p = (curr_ray.ori - volume_nfo.transf.pos) + (volume_nfo.transf.scale * 0.5f); // Relative position from lower object corner
            //uvw = rel_p / volume_nfo.transf.scale; // Normalized UVW
            //uvw_scaled = uvw * geometry::vol::width; // Voxel coordinates! :D

            // Reversed math
            vmath::vec<3> ro = uvw_floored;// + (dir * t.magnitude()); // Apply position delta
            ro /= vol::width; // Back to UVW space (0...1)
            ro *= volume->metadata.transf.scale; // Back to object space
            ro -= volume->metadata.transf.scale * 0.5f; // Position relative to centre, not lower corner
            ro += volume->metadata.transf.pos; // Back to worldspace :D
            ro = vmath::clamp(ro,
                              vmath::vec<3>(volume->metadata.transf.pos - volume->metadata.transf.scale * 0.5f),
                              vmath::vec<3>(volume->metadata.transf.pos + volume->metadata.transf.scale * 0.5f));
            *ro_inout = ro;

            // Return cell discovery state
            // Failure isn't a scary error case anymore - just means that we bounced out of the volume :)
            return cell_found;
        }
        else
        {
            // If we're testing a primary ray, and the current UVW coordinate is filled, we've instantly found a cell :p
            return true;
        }
    }
};

#ifdef GEOMETRY_DBG
#pragma optimize("", on)
#endif
