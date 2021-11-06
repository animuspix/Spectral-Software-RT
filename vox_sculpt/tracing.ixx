export module tracing;

import camera;
import path;
import ui;
import mem;
import vmath;
import parallel;
import vox_ints;
import platform;
import scene;
import sampler;
import aa;

namespace tracing
{
    export path* cameraPaths; // One reusable path/tile for now, minx * miny expected for VCM
                              // (so we can process each one multiple times against arbitrary light paths)
    //path* lightPaths;
    export float* isosurf_distances; // Distances to sculpture boundaries from grid bounds, per-subpixel, refreshed on camera zoom/rotate + animation timesteps (if/when I decide to implement those)
    u32* sample_ctr = nullptr;
    export vmath::vec<2>* tracing_tile_positions = nullptr;
    export vmath::vec<2>* tracing_tile_bounds = nullptr;
    export vmath::vec<2>* tracing_tile_sizes = nullptr;
    export platform::threads::osAtomicInt* completed_tiles;
    export platform::threads::osAtomicInt* tile_prepass_completion;
    platform::threads::osAtomicInt* draws_running;

    export void trace(u16 tilesX, u16 tilesY, u16 tileNdx)
    {
        // Locally cache tile coordinates & extents
        u16 tile_width = (ui::window_width / tilesX);
        u16 tile_height = (ui::window_height / tilesY);
        u16 minX = (tileNdx % tilesX) * tile_width;
        u16 minY = (tileNdx / tilesX) * tile_height;
        u16 xMax = minX + tile_width;
        u16 yMax = minY + tile_height;

        // Make tile coordinates/extents globally visible, to help organize
        // final blitting operations
        tracing_tile_positions[tileNdx].e[0] = minX;
        tracing_tile_positions[tileNdx].e[1] = minY;
        tracing_tile_bounds[tileNdx].e[0] = xMax;
        tracing_tile_bounds[tileNdx].e[1] = yMax;
        tracing_tile_sizes[tileNdx].e[0] = tile_width;
        tracing_tile_sizes[tileNdx].e[1] = tile_height;

        // Local switches to shift between spreading threads over the window to shade the background vs focussing them
        // all on the volume
        bool bg_prepass = true;
        bool volume_tracing = false;

        // Tracing loop
        bool tile_sampling_finished = false;
        while (draws_running->load() > 0)
        {
            if (parallel::tiles[tileNdx].messaging->load() == 0)
            {
                // Stop rendering when we've taken all the image samples we want
                if (sample_ctr[tileNdx] <= aa::max_samples)
                {
                    // If not the background prepass, test if every tile has finished sky sampling before we remap our threads
                    if (!bg_prepass && !volume_tracing)
                    {
                        if (tile_prepass_completion->load() < numTiles)
                        {
                            continue;
                        }
                        else
                        {
                            // Signal that all prepasses have finished, so we can safely move on to sampling our volume grid
                            volume_tracing = true;

                            // Starting to think I should precompute the projected quad, its a lot of math to replicate across every tile
                            /////////////////////////////////////////////////////////////////////////////////////////////////////////////

                            // Resolve worldspace extents for our volume AABB
                            // Will eventually need to adjust this code for different camera angles, zoom, panning, etc.
                            const vmath::vec<3> vol_p = geometry::volume->metadata.transf.pos;
                            const vmath::vec<3> vol_extents = geometry::volume->metadata.transf.scale * 0.5f;

                            // Volume AABB vertices, from left->right, top->bottom, and front->back
                            ///////////////////////////////////////////////////////////////////////

                            // Front
                            const vmath::vec<3> aabb0 = vol_p + vmath(-vol_extents.x(), vol_extents.y, -vol_extents.z);
                            const vmath::vec<3> aabb1 = vol_p + vmath(vol_extents.x(), vol_extents.y, -vol_extents.z);
                            const vmath::vec<3> aabb2 = vol_p + vmath(-vol_extents.x(), -vol_extents.y, -vol_extents.z);
                            const vmath::vec<3> aabb3 = vol_p + vmath(vol_extents.x(), -vol_extents.y, -vol_extents.z);

                            // Back
                            const vmath::vec<3> aabb4 = vol_p + vmath(-vol_extents.x(), vol_extents.y, vol_extents.z);
                            const vmath::vec<3> aabb5 = vol_p + vmath(vol_extents.x(), vol_extents.y, vol_extents.z);
                            const vmath::vec<3> aabb6 = vol_p + vmath(-vol_extents.x(), -vol_extents.y, vol_extents.z);
                            const vmath::vec<3> aabb7 = vol_p + vmath(vol_extents.x(), -vol_extents.y, vol_extents.z);

                            // Project AABB vertices into screenspace
                            const vmath::vec<2> aabb_vertices_ss[8] = { camera::inverse_lens_sample(aabb0),
                                                                        camera::inverse_lens_sample(aabb1),
                                                                        camera::inverse_lens_sample(aabb2),
                                                                        camera::inverse_lens_sample(aabb3),
                                                                        camera::inverse_lens_sample(aabb4),
                                                                        camera::inverse_lens_sample(aabb5),
                                                                        camera::inverse_lens_sample(aabb6),
                                                                        camera::inverse_lens_sample(aabb7) };

                            // Resolve screen-space quad from min/max vertices
                            //////////////////////////////////////////////////

                            // Resolve minimum x, minimum y
                            vmath::vec<4> min_max_px = vmath::vec<4>(9999.9f, 9999.9f, -9999.9f, -9999.9f)
                            for (u8 i = 0; i < 8; i++)
                            {
                                float vx = aabb_vertices_ss[i].e[0], vy = aabb_vertices_ss[i].e[1];
                                min_max_px.e[0] = vmath::fmin(min_max_px.e[0], vx);
                                min_max_px.e[1] = vmath::fmin(min_max_px.e[1], vy);
                                min_max_px.e[2] = vmath::fmax(min_max_px.e[2], vx);
                                min_max_px.e[3] = vmath::fmax(min_max_px.e[3], vy);
                            }

                            // Compute quad width, height
                            const float qWidth = min_max_px.e[2] - min_max_px.e[0];
                            const float qHeight = min_max_px.e[3] - min_max_px.e[1];

                            // Update local tile offsets (x/y bounds, width, height)
                            tile_width = (qWidth / tilesX);
                            tile_height = (qHeight / tilesY);
                            minX = min_max_px.e[0] + ((tileNdx % tilesX) * tile_width);
                            minY = min_max_px.e[1] + ((tileNdx / tilesX) * tile_height);
                            xMax = minX + tile_width;
                            yMax = minY + tile_height;

                            // Update global tile data (accessed on the main thread for blitting operations)
                            tracing_tile_positions[tileNdx].e[0] = minX;
                            tracing_tile_positions[tileNdx].e[1] = minY;
                            tracing_tile_bounds[tileNdx].e[0] = xMax;
                            tracing_tile_bounds[tileNdx].e[1] = yMax;
                            tracing_tile_sizes[tileNdx].e[0] = tile_width;
                            tracing_tile_sizes[tileNdx].e[1] = tile_height;
                        }
                    }

                    // Starting another sample for every pixel in the current tile
                    sample_ctr[tileNdx]++;
                    for (i32 y = minY; y < yMax; y++)
                    {
                        for (i32 x = minX; x < xMax; x++)
                        {
                            // Core path integrator, + demo effects
#define DEMO_SPECTRAL_PT
//#define DEMO_FILM_RESPONSE
//#define DEMO_XOR
//#define DEMO_NOISE
//#define DEMO_AND
//#define DEMO_HYPERBOLA
#ifdef DEMO_HYPERBOLA
                            float distX = ((float)abs(abs(x - ui::image_centre_x) - ui::image_centre_x)) / (float)ui::image_centre_x;
                            float distY = ((float)abs(abs(y - ui::image_centre_y) - ui::image_centre_y)) / (float)ui::image_centre_y;
                            float dist = 1.0f - sqrt(distX * distX + distY * distY);
                            u32 pixel_ndx = y * ui::window_width + x;
                            camera::sensor_response(dist, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                            camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_AND)
                            float rho = (x & y) / (float)y;
                            u32 pixel_ndx = y * ui::window_width + x;
                            camera::sensor_response(rho, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                            camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_XOR)
                            float rho = ((x % 1024) ^ (y % 1024)) / 1024.0f;
                            u32 pixel_ndx = y * ui::window_width + x;
                            camera::sensor_response(rho, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                            camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_NOISE)
                            float sample[4];
                            parallel::rand_streams[tileNdx].next(sample); // Three wasted values :(
                            u32 pixel_ndx = y * ui::window_width + x;
                            camera::sensor_response(sample[0], 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                            camera::tonemap_out(pixel_ndx);
#elif defined (DEMO_FILM_RESPONSE) // Rainbow gradient test
                            u32 pixel_ndx = y * ui::window_width + x;
                            camera::sensor_response((float)x / (float)ui::window_width, 1.0f / aa::max_samples, 1.0f, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                            camera::tonemap_out(pixel_ndx);
#elif defined (DEMO_SPECTRAL_PT)
                            float sample[4];
                            parallel::rand_streams[tileNdx].next(sample); // Random spectral sample, should use QMC here

                            // Intersect the scene/the background
                            u32 pixel_ndx = y * ui::window_width + x;
                            float rho, pdf, rho_weight, power;
                            const path::path_vt cam_vt = camera::lens_sample((float)x, (float)y, sample[0], sample[1], sample[2]);
                            if (!bg_prepass) // If not shading the background, scatter light through the scene
                            {
                                scene::isect(cam_vt,
                                             cameraPaths + tileNdx, isosurf_distances + pixel_ndx, tileNdx);
                                //scene::isect(lights::sky_sample(x, y, sample[0]), lightPaths[tileNdx]);

                                // Integrate scene contributions (unidirectional for now)
                                cameraPaths[tileNdx].resolve_path_weights(&rho, &pdf, &rho_weight, &power); // Light/camera path merging decisions are performed while we integrate camera paths,
                                                                                                            // so we only need to resolve weights for one batch

                                // Remove the current path from the backlog for this tile
                                // (BDPT/VCM implementation will delay this until after separately tracing every path)
                                cameraPaths[tileNdx].clear();
                                //lightPaths[tileNdx].clear();

                                // Test for unresolved/buggy paths
                                //if (cameraPaths[tileNdx].front > 0) platform::osDebugBreak();
                            }
                            else // Otherwise hop directly to the sky
                                 // Similar code to the escaped-path light sampling in [scene.ixx]
                            {
                                vmath::vec<3> ori = cam_vt.ori + (cam_vt.dir * lights::sky_dist);
                                rho = cam_vt.rho_sample;
                                rho_weight = spectra::sky(cam_vt.rho_sample, cam_vt.dir.e[1]);
                                power = cam_vt.power * lights::sky_env(cam_vt.pdf);
                                pdf = cam_vt.pdf;
                            }

                            // Compute sensor response + apply sample weight (composite of integration weight for spectral accumulation,
                            // lens-sampled filter weight for AA, and path index weights from ray propagation)
                            camera::sensor_response(rho, rho_weight, pdf, power, pixel_ndx, sample_ctr[tileNdx]);

                            // Map resolved sensor responses back into tonemapped RGB values we can store for output
                            camera::tonemap_out(pixel_ndx);
#endif
                        }
                    }
                }
                else if (!tile_sampling_finished)
                {
                    if (!bg-prepass)
                    {
                        // Log to console once we finish sampling the volume
                        platform::osDebugLogFmt("%iSPP rendering completed for tile %i\n", aa::max_samples, tileNdx);
                        tile_sampling_finished = true;

                        // Signal the current tile has finished sampling
                        completed_tiles->inc();
                    }
                    else
                    {
                        // Also log after we finish sampling visible sky in the viewport
                        platform::osDebugLogFmt("%iSPP sky prepass completed for tile %i\n", aa::max_samples, tileNdx);

                        // Reset sample counts before we start volume rendering
                        sample_ctr[tileNdx] = 0;

                        // Signal the sky prepass has finished for the current tile
                        prepass_completion_state->inc();
                    }
                }

                // Signal a completed sampling iteration
                parallel::tiles[tileNdx].messaging->inc();
            }
        }
    }
    export void stop_tracing()
    {
        draws_running->store(0);
    }
    export void init()
    {
        // Allocate tracing arrays
        cameraPaths = mem::allocate_tracing<path>(sizeof(path) * parallel::numTiles);
        sample_ctr = mem::allocate_tracing<u32>(sizeof(u32) * parallel::numTiles);
        tracing_tile_positions = mem::allocate_tracing<vmath::vec<2>>(sizeof(vmath::vec<2>) * parallel::numTiles);
        tracing_tile_bounds = mem::allocate_tracing<vmath::vec<2>>(sizeof(vmath::vec<2>) * parallel::numTiles);
        tracing_tile_sizes = mem::allocate_tracing<vmath::vec<2>>(sizeof(vmath::vec<2>) * parallel::numTiles);
        isosurf_distances = (float*)mem::allocate_tracing<float>(sizeof(float) * ui::window_area); // Eventually this will be per-subpixel instead of per-macropixel

        // Resolve tracing types
        platform::osClearMem(cameraPaths, sizeof(path) * parallel::numTiles);
        for (u32 i = 0; i < ui::window_area; i++)
        {
            isosurf_distances[i] = -1.0f; // Reserve zero distance for voxels directly facing a grid boundary
        }

        // Allocate & initialize draw_state
        draws_running = mem::allocate_tracing<platform::threads::osAtomicInt>(sizeof(platform::threads::osAtomicInt));
        draws_running->init();
        draws_running->inc();

        // Allocate & initialize work completion state
        completed_tiles = mem::allocate_tracing<platform::threads::osAtomicInt>(sizeof(platform::threads::osAtomicInt));
        completed_tiles->init();
    }
};