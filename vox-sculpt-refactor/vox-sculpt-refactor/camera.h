
#include "aa.h"
#include "ui.h"
#include "mem.h"
#include "path.h"
#include "vmath.h"
#include "spectra.h"
#include "platform.h"

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
    sensel operator/(const float rhs) const
    {
        return sensel(r / rhs, g / rhs, b / rhs);
    }
    float r;
    float g;
    float b;
};

namespace camera
{
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
    tracing::path_vt lens_sample(uint32_t film_x, uint32_t film_y, float rand_u, float rand_v, float rho);

    // Find the perspective-projected pixel coordinate passing through the given worldspace 3D coordinate
    vmath::vec<2> inverse_lens_sample(vmath::vec<3> world_pos);

    // Combination of a custom sensor response curve (see spectra.h) and a basic integration scheme
    //
    // Integration scheme is very simple for now, just adding filtered colors together up to the max sample count;
    // future versions will use a more sophisticated filter with no decay (so late samples will be implicitly
    // weighted the same as early ones) and I might eventually consider performing integration in spectral space
    // instead (not 100% sure what that would look like)
    void sensor_response(float rho, float rho_weight, float pdf, float power, uint32_t ndx, uint32_t sample_num);

    // Tonemap + write to digital output :)
    void tonemap_out(uint32_t ndx);

    // Initialize
    void init();
}