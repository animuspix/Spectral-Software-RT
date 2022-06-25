export module spectra;
import vox_ints;
import vmath;
import mem;
import platform;

export namespace spectra
{
    // Custom film/sensor response curve, kinda follows the references in Zucconi's diffraction tutorial but heavily iterated to give a more film-like
    // spectrum; Zucconi's tutorial is here
    // https://www.alanzucconi.com/2017/07/15/improving-the-rainbow/,
    // aand my Desmos workspace for the curves themselves is here:
    // https://www.desmos.com/calculator/zm9jwr0ngk
    vmath::vec<3> film(float rho)
    {
        return vmath::vec<3>(vmath::fmax(vmath::quadratic(rho, 4.0f, 0.6f, 0.2f, true), 0.0f) +
                             vmath::fmax(vmath::quadratic(rho, 4.0f, 3.0f, 1.0f, true), 0.0f),
                             vmath::fmax(vmath::gaussian(rho, 1.0f, 0.5f, 0.2f, 0.05f), 0.0f),
                             vmath::fmax(vmath::gaussian(rho, 1.0f, 0.0f, 0.55f, 0.2f) *
                                         vmath::quadratic(rho / 0.4f, 1.0f, -0.6f / 0.4f, -2.3f, false) *
                                         vmath::quadratic(rho, 1.0f, 0.95f, 0.0f, false) + 0.1f, 0.0f));
    }

    // Generic diffuse surface colors until I get imgui working
    const float placeholder_spd(vmath::vec<4> coords_and_rho) // [xyz], w (wavelength)
    {
        return vmath::gaussian(coords_and_rho.w(), 2.7f, 0.8f, 0.2f, 1.7f); // Debug red :p
    }

    // Compressed SPD representation for efficiently sampling (& creating) complex spectra
    // Contains a piecewise curve rasterized to 16 points with 256 fixed-point values each
    export struct RasterizedSPD
    {
        RasterizedSPD() { platform::osClearMem(knots, 16); }

        static constexpr u32 num_spd_samples = 16;
        u8 knots[num_spd_samples];

        // Initialize an SPD from the given continuous distribution
        // (assumed to take scalar frequency samples and return scalar
        // responses, and to be a scalar function - all the variation in
        // response is due to variation in sampled spectrum)
        float initUniformSPD(vmath::fn<0, float> sourceFn)
        {
            float rho = 0;
            for (u32 i = 0; i < num_spd_samples; i++)
            {
                knots[i] = u8(sourceFn(rho) * 255.5f);
                rho += 1.0f / float(num_spd_samples);
            }
        }

        // Initialise just one knot at a time from the SPD - useful for complex SPDs that might vary significantly over space
        // (in which case we'd use one RasterizedSPD for each projected texel, and fill each manually using this function)
        float init_single_knot(float rho, float response) // Rho and Response assumed in 0...1
        {
            knots[u32(rho * num_spd_samples)] = u8(response * 255.5f);
        }

        // Convert sampled spectrum to distribution indices, interpolate, return predicted response
        float sample(float rho) // Rho assumed in 0...1
        {
            const float i = rho * num_spd_samples;
            const float t = i - u32(i);
            float a = float(knots[u32(i)]) / 255.5f;
            float b = float(knots[vmath::umin(u32(i + 1), num_spd_samples - 1)]) / 255.5f;
            return vmath::lerp(a, b, t);
        }
    };

    // Super basic sky model - sharp blue fading to orange as y approaches zero
    // Desmos visualization
    // https://www.desmos.com/calculator/7qqnkr1uqx
    // Eventually this will be updated to use a fancier spectral model, driven by the math here:
    // https://www.scratchapixel.com/lessons/procedural-generation-virtual-worlds/simulating-sky
    float sky(float rho, float y)
    {
        const float blue = vmath::clamp(vmath::gaussian(rho, 0.25f, 0.18f, 0.1f, -0.75f), 0.0f, 1.0f);
        const float orange = vmath::clamp(vmath::gaussian(rho, 0.25f, 0.7f, 0.1f, -0.75f), 0.0f, 1.0f);
        return vmath::lerp(blue, orange, vmath::fabs(y));
    }

    export struct spectral_skybox
    {
        static constexpr float atmos_radius = 1000.0f;
        static constexpr float atmos_diameter = atmos_radius * 2.0f;
        u32 tile_width = 1920; // Skybox tiles are constant 1080p
        u32 tile_height = 1080;
        u32 tile_area = tile_width * tile_height;
        RasterizedSPD* tiled_skybox; // Top, left, right, back, bottom, front (packed contiguously)
        void init()
        {
            tiled_skybox = mem::allocate_tracing<decltype(tiled_skybox)>(sizeof(tiled_skybox) * tile_area * 6); // Six cube faces, each face has fullscreen area
            for (u32 j = 0; j < 6; j++) // For each face
            {
                for (u32 i = 0; i < tile_area; i++) // For each texel
                {
                    for (u32 k = 0; k < RasterizedSPD::num_spd_sasmples; k++) // For each SPD sample
                    {
                        float rho = float(k) / float(RasterizedSPD::num_spd_samples);
                        tiled_skybox[(j * tile_area) + i].init_single_knot(rho, sky(rho, 0.5f)); // Clear blue everywhere, future versions can compute projected sphere positions
                                                                                                   // for accurate sampling
                    }
                }
            }
        }

        // Regenerate the skybox with altered sun position, atmospheric density, atmospheric radius, etc.
        void regenerate()
        {
            // Does nothing for now...
        }

        // Needs to map the given ray origin/direction onto a skybox texel, find response for the given spectrum, and return
        float sample(vmath::vec<3> ray_origin, vmath::vec<3> ray_direction, float rho)
        {
            // Assume ray origins are consistently close to the centre of the atmosphere
            vmath::vec<3> worldspace_texel_p = ray_origin + ray_direction * atmos_radius;

            // Compute the tile to sample here...
            // Should be similar/equivalent to computing normals on a cube
            // Quite tempting just to intersect planes one-by-one ^_^', but we shouldn't, this code will be called at least once for every camera sensor
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // Convert to image space with an inverse perspective projection
            // Need to rotate the projection to the proper tile first >.>
            // Doable by keeping the projection the same and rotating the worldspace texel (very @.@)
            // In that case we'd want to rotate the worldspace texel towards the front skybox plane
            vmath::vec<2> uv = vmath::inverse_perspective_projection(worldspace_texel_p, tile_width / 2, tile_height / 2, atmos_radius);

            // Sample ^_^
            // Hardcoded front tile atm
            u32 ndx = ((uv.y() * tile_height) * tile_width) + (uv.x() * tile_width);
            return tiled_skybox[(5 * tile_area) + ndx].sample(rho);
        }
    };

    // Stratified spectral representation for camera sampling
    struct spectral_buckets
    {
        static constexpr u32 num_buckets = 16;
        static constexpr float interval_size = 1.0f / num_buckets;
        u32 last_bucket = 0;
        float buckets[num_buckets] = {};
        void init()
        {
            // Random bucket selection by default
            for (u32 i = 0; i < num_buckets; i++)
            {
                buckets[i] = 0.0f;
            }
        }
        float draw_sample(float u, float v)
        {
            // Try to select a frequency from a curve skewed towards colors reflected most by the environment
            // This should work for most scenes
            for (u32 n = 0; n < num_buckets; n++)
            {
                if (u < buckets[n]) // Buckets with higher sensitivities will naturally pass this test more often over time
                {
                    last_bucket = n; // Record the selected bucket so we can update it with a weight after tracing
                    return (n * interval_size) + (v * interval_size); // Return a random offset within the selected bucket
                }
            }

            // The surface is matte black; all frequencies reflected equally poorly
            // Since no frequencies are particularly bright, just return one at random
            last_bucket = static_cast<u32>(vmath::ffloor(u * num_buckets));
            return (last_bucket * interval_size) + (v * interval_size);
        }
        void update(float weight)
        {
            buckets[last_bucket] = weight; // Any noise from using per-sample weights seems to be cancelled out during filtering, and
                                           // naively smoothing here (e.g. by using a weight / num_samples box filter) seems to damage the
                                           // importance function (by skewing weights down) more than it helps account for natural spectral
                                           // variance
        }
    };
};
