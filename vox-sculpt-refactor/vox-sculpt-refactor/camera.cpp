#include "camera.h"

// Intermediate sensor values for filtering/anti-aliasing/temporal integration
sensel* sensor_grid;
constexpr uint32_t sensor_grid_footprint = ui::window_width * ui::window_height * sizeof(sensel);

// Screen color/picture data (8bpc)
uint32_t* digital_colors;
constexpr uint32_t digital_colors_footprint = ui::window_width * ui::window_height * sizeof(uint32_t);

// Sum of the filter values used for each sample, per-sensor
// Needed for correct image integration when I'm using non-boxy filters, since those filters
// can integrate to more/less than 1 and (=> and lead to weird results unless you divide out
// the sum of the filter values each frame)
float* filter_sum_grid;
constexpr uint32_t filter_sum_grid_footprint = ui::window_width * ui::window_height * sizeof(float);

// Camera sampling! just perspective projection for now :)
constexpr float FOV_RADS = vmath::pi * 0.5f;
const vmath::vec<3> camera_pos() { return vmath::vec<3>(0, 0, -10.0f); }
const float camera_z_axis() { return (ui::window_width * aa::samples_x) / vmath::ftan(FOV_RADS * 0.5f); }

tracing::path_vt camera::lens_sample(uint32_t film_x, uint32_t film_y, float rand_u, float rand_v, float rho)
{
    // Compute supersampled coordinates
    vmath::vec<2> film_xy = aa::supersample(float(film_x), float(film_y));

    // Compute local sample coordinate
    const vmath::vec<2> subpx_xy = aa::jitter(rand_u, rand_v);

    // Compute filter weight, accumulate for the sampled sensor
    float filt = aa::blackman_harris_weight(subpx_xy);
    filter_sum_grid[film_x + film_y * ui::window_width] += filt;

    // Apply sample offset into film offset
    film_xy += subpx_xy;

    // Return weighted lens direction :D
    const vmath::vec<3> c = camera_pos();
    // Account for camera rotation here...
    return tracing::path_vt(vmath::vec<3>(film_xy.x() - ui::image_centre_x * aa::samples_x,
        film_xy.y() - ui::image_centre_y * aa::samples_y,
        camera_z_axis()).normalized(), // Probably don't need to normalize here, but the stability feels nice
        c, 1.0f, rho, 1.0f, 1.0f * filt);
}

vmath::vec<2> camera::inverse_lens_sample(vmath::vec<3> world_pos)
{
    // This algorithm traps world_pos within the projected camera vertices, and assumes a pinhole camera
    // This works because rays within the projected lens stay within the distribution of our lens for the
    // given FOV. Previous versions traced worldspace points back to their intersection with the camera's
    // initial projection plane (at camera_z_axis()), which proeduced distortion since there was no
    // guarantee the line between world points and camera pixels would be parallel with camera rays
    // intersecting the same points (especially for very wide or very narrow FOV)
    // Anither issue was that pixels never actually sit on the camera plane - they're created out there,
    // then immediately normalizeed, which would be fine if the rays projected back from our worldspace
    // points were coherent with the frustum, but as above, they could easily not be - so hidden points
    // could still read as visible after stretching them to the camera plane, and then map onto invalid
    // pixels when the gradient towards them is much lower/higher than the gradient of the nearest camera
    // ray (causing the stretch to displace their x/y coordinates less than it would have otherwise)
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    // Define each camera corner (normalized, giving a direction from the camera origin)
    int32_t samples_x = int32_t(aa::samples_x);
    int32_t samples_y = int32_t(aa::samples_y);
    vmath::vec<3> corners[4] = { vmath::vec<3>(-ui::image_centre_x * samples_x, ui::image_centre_y * samples_y, camera_z_axis()).normalized(), // Upper-left
                                 vmath::vec<3>(ui::image_centre_x * samples_x, ui::image_centre_y * samples_y, camera_z_axis()).normalized(), // Upper-right
                                 vmath::vec<3>(-ui::image_centre_x * samples_x, -ui::image_centre_y * samples_y, camera_z_axis()).normalized(), // Lower-left
                                 vmath::vec<3>(ui::image_centre_x * samples_x, -ui::image_centre_y * samples_y, camera_z_axis()).normalized() }; // Lower-right

    // Extend corners out to the z-plane containing world_pos
    for (uint32_t i = 0; i < 4; i++)
    {
        float dx = corners[i].x() / corners[i].z();
        float dy = corners[i].y() / corners[i].z();
        corners[i].e[0] = dx * world_pos.z();
        corners[i].e[1] = dy * world_pos.z();
        corners[i].e[2] = world_pos.z();
    }

    // Compute world_pos position relative to the lower-right corner of the projected lens
    vmath::vec<3> rel_p0 = world_pos - corners[2];

    // Normalize relative XY coordinates
    vmath::vec<2> wh = vmath::vabs(corners[0].xy() - corners[3].xy());
    vmath::vec<2> uv = rel_p0.xy() / wh;

    // Return scaled coordinates
    return vmath::vec<2>(uv.x() * ui::window_width,
        uv.y() * ui::window_height);
}

void camera::sensor_response(float rho, float rho_weight, float pdf, float power, uint32_t ndx, uint32_t sample_num)
{
    // Resolve responses per-channel
    vmath::vec<3> rgb = spectra::film(rho) * rho_weight * (power / pdf);

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

void camera::tonemap_out(uint32_t ndx)
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
    sensel sensor_v = sensor_grid[ndx] / filter_sum_grid[ndx];
    const uint32_t r = static_cast<uint32_t>(aces(vmath::clamp(sensor_v.r, 0.0f, 1.0f)) * 256);
    const uint32_t g = static_cast<uint32_t>(aces(vmath::clamp(sensor_v.g, 0.0f, 1.0f)) * 256);
    const uint32_t b = static_cast<uint32_t>(aces(vmath::clamp(sensor_v.b, 0.0f, 1.0f)) * 256);
    const uint32_t a = 0x0;
    digital_colors[ndx] = b |
        (g << 8) |
        (r << 16) |
        (a << 24);
}

void camera::init()
{
    //assert(mem::tracing_arena != nullptr);
    sensor_grid = mem::allocate_tracing<sensel>(sensor_grid_footprint);
    digital_colors = mem::allocate_tracing<uint32_t>(digital_colors_footprint);
    filter_sum_grid = mem::allocate_tracing<float>(filter_sum_grid_footprint);
    platform::osClearMem(filter_sum_grid, filter_sum_grid_footprint);
}
