
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

class camera
{
public:
    static tracing::path_vt lens_sample(uint32_t film_x, uint32_t film_y, float rand_u, float rand_v, float rho);

    // Find the perspective-projected pixel coordinate passing through the given worldspace 3D coordinate
    static vmath::vec<2> inverse_lens_sample(vmath::vec<3> world_pos);

    // Combination of a custom sensor response curve (see spectra.h) and a basic integration scheme
    //
    // Integration scheme is very simple for now, just adding filtered colors together up to the max sample count;
    // future versions will use a more sophisticated filter with no decay (so late samples will be implicitly
    // weighted the same as early ones) and I might eventually consider performing integration in spectral space
    // instead (not 100% sure what that would look like)
    static void sensor_response(float rho, float rho_weight, float pdf, float power, uint32_t ndx, uint32_t sample_num);

    // Tonemap + return for digital output :)
    static uint32_t tonemap_out(uint32_t ndx);

    // Initialize
    static void init();
};