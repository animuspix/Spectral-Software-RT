export module camera;

import ui;
import mem;
import path;
import vmath;
import scene;
import spectra;
import vox_ints;
import aa;

// Digital color/sensor data model, used to map from spectral inputs to writable screen colors
// (slowly turning into a retinal cone cell model instead oops, might update names eventually)
struct sensel
{
    sensel(float _r, float _g, float _b) :
        r(_r), g(_g), b(_b) {}
    sensel operator+(const sensel& rhs)
    {
        return sensel(rhs.r + r, rhs.g + g, rhs.b + b);
    }
    sensel operator*(const float rhs) const
    {
        return sensel(r * rhs, g * rhs, b * rhs);
    }
    float r;
    float g;
    float b;
};

export namespace camera
{
    // Intermediate sensor values for filtering/anti-aliasing/temporal integration
    sensel* sensor_grid;
    constexpr u32 sensor_grid_footprint = ui::window_width * ui::window_height * sizeof(sensel);

    // Screen color/picture data (8bpc)
    u32* digital_colors;
    constexpr u32 digital_colors_footprint = ui::window_width * ui::window_height * sizeof(u32);

    // Camera sampling! just perspective projection for now :)
    const vmath::vec<3> camera_pos() { return vmath::vec<3>(0, 0, -10.0f); }
    constexpr float FOV_RADS = vmath::pi * 0.5f;
    export tracing::path_vt lens_sample(float film_x, float film_y, float rand_u, float rand_v, float rho)
    {
        // Jitter supersampled coordinates
        const vmath::vec<2> film_p_aa = aa::jitter(film_x,
                                                   film_y,
                                                   rand_u, rand_v);
        // Compute filter weight
        float filt = aa::blackman_harris_weight(film_p_aa - vmath::vec<2>(film_x * aa::samples_x, film_y * aa::samples_y));

        // Return weighted lens direction :D
        const vmath::vec<3> c = camera_pos();
        return tracing::path_vt(vmath::vec<3>(film_p_aa.x() - ui::image_centre_x * aa::samples_x,
                                              film_p_aa.y() - ui::image_centre_y * aa::samples_y,
                                              (ui::window_width * aa::samples_x) / vmath::ftan(FOV_RADS * 0.5f)).normalized(), // Probably don't need to normalize here, but the stability feels nice
                                              c, filt, rho, 1.0f);
    }

    // Combination of a custom sensor response curve (see spectra.h) and a basic integration scheme
    //
    // Integration scheme is very simple for now, just adding filtered colors together up to the max sample count;
    // future versions will use a more sophisticated filter with no decay (so late samples will be implicitly
    // weighted the same as early ones) and I might eventually consider performing integration in spectral space
    // instead (not 100% sure what that would look like)
    void sensor_response(float rho, float rho_weight, float pdf, float power, u32 ndx, u32 sample_num)
    {
        // Resolve responses per-channel
        vmath::vec<3> rgb = spectra::film(rho * rho_weight) * pdf * power;

        // Compose isolated colours into a sensor value, apply accumulated weights
        // (from path-tracing + spectral integration), write to sensor output :)
        sensel& curr_sensel = sensor_grid[ndx]; // Read in current sensor value
        if (sample_num == 1)
        {
            curr_sensel.r = rgb.e[0];
            curr_sensel.g = rgb.e[1];
            curr_sensel.b = rgb.e[2];
        }
        else if (sample_num < aa::max_samples)
        {
            curr_sensel.r += rgb.e[0];
            curr_sensel.g += rgb.e[1];
            curr_sensel.b += rgb.e[2];
        }
    }

    // Tonemap + write to digital output :)
    export void tonemap_out(u32 ndx)
    {
        // Tonemap into integer 8bpc + return
        // Tonemapped with ACES, source:
        // https://www.shadertoy.com/view/WdjSW3
        // originally Narkowicz 2015, "ACES Filmic Tone Mapping Curve"
        auto aces = [](float x) {
            const float a = 2.51f;
            const float b = 0.03f;
            const float c = 2.43f;
            const float d = 0.59f;
            const float e = 0.14f;
            return (x * (a * x + b)) / (x * (c * x + d) + e);
        };
        sensel sensor_v = sensor_grid[ndx];
        const u32 r = static_cast<u32>(aces(vmath::clamp(sensor_v.r, 0.0f, 1.0f)) * 256);
        const u32 g = static_cast<u32>(aces(vmath::clamp(sensor_v.g, 0.0f, 1.0f)) * 256);
        const u32 b = static_cast<u32>(aces(vmath::clamp(sensor_v.b, 0.0f, 1.0f)) * 256);
        const u32 a = 0x0;
        digital_colors[ndx] = b |
                             (g << 8) |
                             (r << 16) |
                             (a << 24);
    }

    // Initialize
    void init()
    {
        //assert(mem::tracing_arena != nullptr);
        sensor_grid = mem::allocate_tracing<sensel>(sensor_grid_footprint);
        digital_colors = mem::allocate_tracing<u32>(digital_colors_footprint);
    }
}