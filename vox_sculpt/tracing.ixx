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
import geometry;
import spectra;
import sampler;
import lights;
import aa;

//#define TRACING_DBG
#ifdef TRACING_DBG
#pragma optimize("", off)
#endif

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

    // Stratified spectra per-pixel, allowing us to bias color samples towards the scene SPD instead of drawing randomly for every tap
    spectra::spectral_buckets* spectral_strata = nullptr;

    // Allow different "render modes" for different user activities (editing, previews, final output to file)
    // Each render-mode uses a specialized integrator; the "final" modes are heavily pipelined and designed to be one-offs,
    // the TO_FILE mode has no communication with regular window paints + copies output directly to an image buffer, and
    // the EDIT mode renders in discrete, nearly stateless frames (so that it can respond immediately to changes in the
    // volume, camera position, etc)
    export enum RENDER_MODES
    {
        RENDER_MODE_EDIT,
        RENDER_MODE_FINAL_PREVIEW,
        RENDER_MODE_FINAL_TO_FILE
        // RENDER_MODE_PAINTING
        // RENDER_MODE_SCULPTING
        // ...etc...
    };

    // Set the render mode to use in the next "frame"
    RENDER_MODES renderMode = RENDER_MODE_EDIT; // Edit mode by default
    RENDER_MODES lastRenderMode = RENDER_MODE_EDIT;
    export void set_render_mode(RENDER_MODES mode)
    {
        lastRenderMode = renderMode;
        renderMode = mode;
    }

    // Clear render state (needed for render mode transitions + refreshing before each EDIT pass)
    void clear_render_state(u32 tileNdx, i32 minX, i32 xMax, i32 minY, i32 yMax)
    {
        sample_ctr[tileNdx] = 0;
        camera::clear_patch(minX, xMax, minY, yMax);
        platform::osClearMem(cameraPaths, sizeof(path) * parallel::numTiles);
        for (i32 i = 0; i < ui::window_area; i++)
        {
            *(isosurf_distances + i) = -1.0f;
            spectral_strata[i].init();
        }
    }

    // Core image integrator - shared across all render modes for simplicity
    // The idea is that every mode gets here eventually, but they vary in how pipelined they are and whether they send
    // updates to the window
    void core_image_integrator(u32 tileNdx, i32 stride, i32 minY, i32 yMax, i32 minX, i32 xMax, bool bg_prepass)
    {
        // Starting another sample for every pixel in the current tile
        sample_ctr[tileNdx]++;
        i32 dy = stride;
        bool ylerp = true; // Interpolate stridden lines by default
        for (i32 y = minY; y < yMax; y += dy)
        {
            // Sample pixels in the current row
            i32 dx = stride;
            bool xlerp = true; // Interpolate stridden tiles by default
            for (i32 x = minX; x < xMax; x += dx)
            {
                // Core path integrator, + demo effects
                u32 pixel_ndx = y * ui::window_width + x;
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
                camera::sensor_response(dist, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_AND)
                float rho = (x & y) / (float)y;
                camera::sensor_response(rho, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_XOR)
                float rho = ((x % 1024) ^ (y % 1024)) / 1024.0f;
                camera::sensor_response(rho, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_NOISE)
                float sample[4];
                parallel::rand_streams[tileNdx].next(sample); // Three wasted values :(
                camera::sensor_response(sample[0], 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                camera::tonemap_out(pixel_ndx);
#elif defined (DEMO_FILM_RESPONSE) // Rainbow gradient test
                camera::sensor_response((float)x / (float)ui::window_width, 1.0f / aa::max_samples, 1.0f, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
                camera::tonemap_out(pixel_ndx);
#elif defined (DEMO_SPECTRAL_PT)
                            // Draw random values for lens sampling
                float sample[4];
                parallel::rand_streams[tileNdx].next(sample);

                // Resolve a spectral sample for the current pixel
                float s = spectral_strata[pixel_ndx].draw_sample(sample[0], sample[1]);

                // Intersect the scene/the background
                float rho, pdf, rho_weight, power;
                const path_vt cam_vt = camera::lens_sample((float)x, (float)y, sample[2], sample[3], s);
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
                    power = cam_vt.power * lights::sky_env(&pdf);
                    //pdf = cam_vt.pdf;
                }

                // Compute sensor response + apply sample weight (composite of integration weight for spectral accumulation,
                // lens-sampled filter weight for AA, and path index weights from ray propagation)
                camera::sensor_response(rho, rho_weight, pdf, power, pixel_ndx, sample_ctr[tileNdx]);

                // Map resolved sensor responses back into tonemapped RGB values we can store for output
                camera::tonemap_out(pixel_ndx);

                // Update weight for the bucket containing the current spectral sample
                spectral_strata[pixel_ndx].update(rho_weight);
#endif
                // Blend stridden pixels after each tap (x-component)
                if (x > minX && stride > 1 && xlerp) camera::reconstruct_x(x, pixel_ndx, stride);

                // Modify d-x to avoid skipping the final column in each tile
                if ((xMax - x) <= stride)
                {
                    dx = 1;
                    xlerp = false; // Switch interpolation off if we've stopped striding between pixels
                }
            }

            // Blend stridden pixels after each tap (y-component)
            if (y > minY && stride > 1 && ylerp) camera::reconstruct_y(y, stride);

            // Modify d-y to avoid skipping the final row in each tile
            if ((yMax - y) <= stride)
            {
                dy = 1;
                ylerp = false; // Switch interpolation off if we've stopped striding between lines
            }
        }
    }

    export void trace(u16 tilesX, u16 tilesY, u16 tileNdx)
    {
        // Locally cache tile coordinates & extents
        i32 numTiles = tilesX * tilesY;
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

        // Adjustable sensel stride for undersampling; helps with performance in debug renders
        // Sensels skipped by sample stride are approximated using bilinear interpolation (see camera.ixx)
        // Stride in practice is equal to the number of pixels skipped + 1 (so zero pixels is stride 1, one pixel is stride 2, etc)
        // Thinking of modifying the flow to help with readability there
        constexpr u8 stride = 4; // Eventually this will be driven by imgui

        // Local switches to shift between spreading threads over the window to shade the background vs focussing them
        // all on the volume
        bool bg_prepass = true;
        bool volume_tracing = false;

        // Tracing loop
        bool tile_sampling_finished = false;
        bool draws_running_local = true;
        bool samples_processed = false;
        u32 tracing_thread_ticks = 0;
        u32 sample_interval = 20; // Arbitrary initial value
        double tick_timepoint = 0;
        double sample_timings = 0;
        bool tile_resolved = false;
        while (draws_running_local)
        {
            // Clear render state if the render-mode has recently changed
            bool render_state_cleared = false; // Flag to avoid clearing render state twice in one frame, since its an expensive process
            if (lastRenderMode != renderMode)
            {
                clear_render_state(tileNdx, minX, xMax, minY, yMax);
                render_state_cleared = true;
                lastRenderMode = renderMode;
            }

            // Avoid processing tiles once all samples have resolved (final render modes only)
            if (renderMode == RENDER_MODE_FINAL_PREVIEW || renderMode == RENDER_MODE_FINAL_TO_FILE)
            {
                if (tile_resolved) continue;
            }

            // Test running state at fixed intervals to reduce atomic calls
            if (tracing_thread_ticks % sample_interval == 0)
            {
                draws_running_local = draws_running->load() > 0;
                samples_processed = parallel::tiles[tileNdx].messaging->load() > 0;
                double ms = platform::osGetCurrentTimeMilliSeconds();
                sample_timings = ms - tick_timepoint;
                tick_timepoint = ms;

                // Fit the sampling interval to the time taken per-tick, targeting 30ms/30fps
                if (sample_timings < 30) sample_interval++;
                else sample_interval--;
                sample_interval = vmath::umax(sample_interval, 2u); // Because our sampling interval isn't exclusively affected by the number of ticks we use -
                                                                    // we still need to compute the samples themselves. We can't guarantee that the sampling process
                                                                    // will always take under 30ms, and if we ignore it then our sample interval is likely to underflow
                                                                    // (since it could decrement an infinite number of times)
                                                                    // So we give ourselves at least two ticks after finalizing each sample, and if we're lucky enough
                                                                    // to need more we can have them :D

                // Set a max value for our sample intervals as well, to prevent them spiralling off and overflowing that way
                sample_interval = 1;//vmath::umin(sample_interval, 0x00f00000);
            }
            if (!samples_processed)
            {
                if (renderMode == RENDER_MODE_FINAL_PREVIEW || renderMode == RENDER_MODE_FINAL_TO_FILE)
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

                                // Compute quad width, height
                                const u16 qWidth = static_cast<u16>(vmath::fabs(geometry::vol::metadata->transf.ss_v3.x() - geometry::vol::metadata->transf.ss_v0.x()));
                                const u16 qHeight = static_cast<u16>(vmath::fabs(geometry::vol::metadata->transf.ss_v3.y() - geometry::vol::metadata->transf.ss_v0.y()));

                                // Update local tile offsets (x/y bounds, width, height)
                                tile_width = (qWidth / tilesX);
                                tile_height = (qHeight / tilesY);
                                minX = static_cast<u16>(geometry::vol::metadata->transf.ss_v0.x() + ((tileNdx % tilesX) * tile_width));
                                minY = static_cast<u16>(geometry::vol::metadata->transf.ss_v0.y() + ((tileNdx / tilesX) * tile_height));
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

                        // Invoke our core image integrator
                        // (shared across render modes for ease of development)
                        core_image_integrator(tileNdx, stride, minY, yMax, minX, xMax, bg_prepass);
                    }
                    else if (!tile_sampling_finished)
                    {
                        if (!bg_prepass)
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
                            tile_prepass_completion->inc();
                            bg_prepass = false;
                        }
                    }
                    else
                    {
                        tile_resolved = true;
                    }

                    // Signal a completed sampling iteration
                    parallel::tiles[tileNdx].messaging->store(1);
                    samples_processed = true;
                    tick_timepoint = platform::osGetCurrentTimeMilliSeconds();
                }
                else if (renderMode == RENDER_MODE_EDIT)
                {
                    // Reset render state
                    if (!render_state_cleared)
                    {
                        clear_render_state(tileNdx, minX, xMax, minY, yMax);
                    }

                    // Subtle difference from final modes - edit mode requires taking all samples before passing images back to the window
                    while (sample_ctr[tileNdx] < aa::max_samples)
                    {
                        core_image_integrator(tileNdx, stride, minY, yMax, minX, xMax, false); // No pre-passes in edit mode
                    }

                    // Signal a completed sampling iteration
                    parallel::tiles[tileNdx].messaging->store(1);
                    samples_processed = true;
                    tick_timepoint = platform::osGetCurrentTimeMilliSeconds();
                }
                tracing_thread_ticks++;
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

        // Allocate & initialize spectral strata
        spectral_strata = mem::allocate_tracing<spectra::spectral_buckets>(sizeof(spectra::spectral_buckets) * ui::window_area);
        for (u32 i = 0; i < ui::window_area; i++)
        {
            spectral_strata[i].init(); // Reserve zero distance for voxels directly facing a grid boundary
        }

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
        tile_prepass_completion = mem::allocate_tracing<platform::threads::osAtomicInt>(sizeof(platform::threads::osAtomicInt));
        tile_prepass_completion->init();
    }
};

#ifdef TRACING_DBG
#pragma optimize("", on)
#endif
