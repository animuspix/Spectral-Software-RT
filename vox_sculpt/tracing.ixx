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
    //path lightPaths[parallel::numTiles];
    export float* isosurf_distances; // Distances to sculpture boundaries from grid bounds, per-subpixel, refreshed on camera zoom/rotate + animation timesteps (if/when I decide to implement those)
    u32 sample_ctr[parallel::numTiles] = {};
    export vmath::vec<2> tracing_tile_positions[parallel::numTiles] = {};
    export vmath::vec<2> tracing_tile_bounds[parallel::numTiles] = {};
    export vmath::vec<2> tracing_tile_sizes[parallel::numTiles] = {};
    platform::threads::osAtomicInt* draws_running;

    export void trace(u16 tilesX, u16 tilesY, u16 tileNdx)
    {
        // Locally cache tile coordinates & extents
        const u16 tile_width = (ui::window_width / tilesX);
        const u16 tile_height = (ui::window_height / tilesY);
        const u16 minX = (tileNdx % tilesX) * tile_width;
        const u16 minY = (tileNdx / tilesX) * tile_height;
        const u16 xMax = minX + tile_width;
        const u16 yMax = minY + tile_height;

        // Make tile coordinates/extents globally visible, to help organize
        // final blitting operations
        tracing_tile_positions[tileNdx].e[0] = minX;
        tracing_tile_positions[tileNdx].e[1] = minY;
        tracing_tile_bounds[tileNdx].e[0] = xMax;
        tracing_tile_bounds[tileNdx].e[1] = yMax;
        tracing_tile_sizes[tileNdx].e[0] = tile_width;
        tracing_tile_sizes[tileNdx].e[1] = tile_height;

        // Tracing loop
        bool tile_sampling_finished = false;
        while (draws_running->load() > 0)
        {
            if (parallel::tiles[tileNdx].status->load() == 0)
            {
                // Stop rendering when we've taken all the image samples we want
                if (sample_ctr[tileNdx] <= aa::max_samples)
                {
                    sample_ctr[tileNdx]++; // Starting another sample for every pixel in the current tile
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
                            camera::sensor_response((float)x / (float)ui::window_width, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                            camera::tonemap_out(pixel_ndx);
#elif defined (DEMO_SPECTRAL_PT)
                            float sample[4];
                            parallel::rand_streams[tileNdx].next(sample); // Random spectral sample, should use QMC here

                            // Intersect the scene
                            u32 pixel_ndx = y * ui::window_width + x;
                            scene::isect(camera::lens_sample((float)x, (float)y, sample[0], sample[1], sample[2]),
                                         cameraPaths + tileNdx, isosurf_distances + pixel_ndx, tileNdx);
                            //scene::isect(lights::sky_sample(x, y, sample[0]), lightPaths[tileNdx]);

                            // Integrate scene contributions (unidirectional for now)
                            float rho, pdf, rho_weight;
                            cameraPaths[tileNdx].resolve_path_weights(&rho, &pdf, &rho_weight); // Light/camera path merging decisions are performed while we integrate camera paths,
                                                                                            // so we only need to resolve weights for one batch

                            // Apply path probability into sample weight
                            rho_weight *= pdf;

                            // Remove the current path from the backlog for this tile
                            // (BDPT/VCM implementation will delay this until after separately tracing every path)
                            cameraPaths[tileNdx].clear();
                            //lightPaths[tileNdx].clear();

                            // Test for unresolved/buggy paths
                            //if (cameraPaths[tileNdx].front > 0) platform::osDebugBreak();

                            // Compute sensor response + apply sample weight (composite of integration weight for spectral accumulation,
                            // lens-sampled filter weight for AA, and path index weights from ray propagation)
                            camera::sensor_response(rho, rho_weight, pixel_ndx, sample_ctr[tileNdx]);

                            // Map resolved sensor responses back into tonemapped RGB values we can store for output
                            camera::tonemap_out(pixel_ndx);
                        }
#endif
                    }
                }
                else if (!tile_sampling_finished)
                {
                    // Log to console once we finish sampling
                    platform::osDebugLogFmt("%iSPP rendering completed for tile %i\n", aa::max_samples, tileNdx);
                    tile_sampling_finished = true;
                }
                // Signal a completed sampling iteration
                parallel::tiles[tileNdx].status->inc();
            }
        }
    }
    export void stop_tracing()
    {
        draws_running->store(0);
    }
    export void init()
    {
        cameraPaths = mem::allocate_tracing<path>(sizeof(path) * parallel::numTiles);
        platform::osClearMem(cameraPaths, sizeof(path) * parallel::numTiles);
        isosurf_distances = (float*)mem::allocate_tracing<float>(sizeof(float) * ui::window_area); // Eventually this will be per-subpixel instead of per-macropixel, but I'm not quite up to adding AA yet
        for (u32 i = 0; i < ui::window_area; i++)
        {
            isosurf_distances[i] = -1.0f; // Reserve zero distance for voxels directly facing a grid boundary
        }

        // Allocate & initialize draw_state
        draws_running = mem::allocate_tracing<platform::threads::osAtomicInt>(sizeof(platform::threads::osAtomicInt));
        draws_running->init();
        draws_running->inc();
    }
};