#pragma once

#include <stdint.h>
#include "vmath.h"
#include "materials.h"

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

namespace vol_constants
{
    static constexpr uint32_t width = 1024; // All volumes are 1024 * 1024 * 1024
    static constexpr uint32_t slice_area = width * width;
    static constexpr uint32_t res = slice_area * width;
    static constexpr float cell_size = 1.0f / width;
    static constexpr uint32_t max_cell_ndx_per_axis = width - 1;
};

class geometry
{
public:
	static void init(vmath::vec<2>(*inverse_lens_sampler_fn)(vmath::vec<3>));
	static void spin(float xrot, float yrot);
	static void zoom(float z);
	static bool test(vmath::vec<3> dir, vmath::vec<3>* ro_inout, vol_nfo* vol_nfo_out);
	static bool cell_step(vmath::vec<3> dir, vmath::vec<3>* ro_inout, vmath::vec<3> uvw_in, vmath::vec<3, int32_t>* uvw_i_inout, vmath::vec<3>* n_out, bool primary_ray);
};