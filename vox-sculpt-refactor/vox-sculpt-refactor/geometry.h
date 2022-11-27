#pragma once

#include <stdint.h>
#include "vmath.h"
#include "materials.h"

class geometry
{
public:
    struct vol
    {
        static constexpr uint32_t width = 1024; // All volumes are 1024 * 1024 * 1024
        static constexpr uint32_t slice_area = width * width;
        static constexpr uint32_t res = slice_area * width;
        static constexpr float cell_size = 1.0f / width;
        static constexpr uint32_t max_cell_ndx_per_axis = width - 1;
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
        static vol_nfo* metadata;

        // Our geometry is composed of individual bits, grouped into 64-bit chunks;
        // metachunks take that abstraction one layer higher by providing groups of 2x2x2
        // chunks for efficient traversal (one metachunk is one cacheline)
        // Individual voxels (within chunks) occupy one bit each; order is left-right/front-back/top-bottom
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
        // We evaluate these cells by constructing a 64-bit mask for the one we want to test; if the current chunk AND the mask is nonzero, we have a set cell, otherwise we have an empty one
        static uint8_t* metachunk_occupancies; // Direct mask of occupancies per-metachunk, for faster testing during chunk/metachunk traversal
        struct metachunk
        {
            // Metachunk dimensions in chunks
            static constexpr uint32_t res_x = 2;
            static constexpr uint32_t res_y = 2;
            static constexpr uint32_t res_z = 2;
            static constexpr uint32_t res_xy = res_x * res_y;
            static constexpr uint32_t res = res_xy * res_z;

            // Chunk dimensions in voxels
            static constexpr uint32_t chunk_res_x = 4;
            static constexpr uint32_t chunk_res_y = 4;
            static constexpr uint32_t chunk_res_z = 4;
            static constexpr uint32_t chunk_res_xy = chunk_res_x * chunk_res_y;

            // Metachunk dimensions in voxels
            static constexpr uint32_t num_vox_x = res_x * chunk_res_x;
            static constexpr uint32_t num_vox_y = res_y * chunk_res_y;
            static constexpr uint32_t num_vox_z = res_z * chunk_res_z;
            static constexpr uint32_t num_vox_xy = num_vox_x * num_vox_y;
            static constexpr uint32_t num_vox = num_vox_xy * num_vox_z;

            // Metachunk data
            uint64_t chunks[res];
        };

        static constexpr uint32_t num_metachunks_x = width / metachunk::num_vox_x;
        static constexpr uint32_t num_metachunks_y = width / metachunk::num_vox_y;
        static constexpr uint32_t num_metachunks_z = width / metachunk::num_vox_z;
        static constexpr uint32_t num_metachunks_xy = num_metachunks_x * num_metachunks_y;
        static constexpr uint32_t num_metachunks = num_metachunks_xy * num_metachunks_z;
        static metachunk* metachunks;

        static uint32_t chunk_index_solver(vmath::vec<3, int32_t> uvw_floored); // Returns chunk index
        static uint32_t metachunk_index_solver(vmath::vec<3, int32_t> uvw_floored);
        static uint32_t metachunk_index_solver_fast(vmath::vec<3, int32_t> metachunk_uvw); // Returns metachunk index

        struct voxel_ndces
        {
            uint8_t chunk;
            uint64_t bitmask;
        };

        static voxel_ndces voxel_index_solver(vmath::vec<3, int32_t> uvw_floored); // Returns metachunk + chunk + bitmask to select specific voxels within chunks
                                                                                  // [uvw_floored] is expected in voxel space (0...vol::width on each axis)

        // Resolve screen-space volume bounds for the current camera transform
        // (inverse lens sampling performed on the camera, so this just needs to resolve worldspace bounds and reproject them that way, before
        // taking the min/max coordinates in the 2D plane and storing those)
        static void resolveSSBounds(vmath::vec<2>(*inverse_lens_sampler_fn)(vmath::vec<3>));
    };

	static void init(vmath::vec<2>(*inverse_lens_sampler_fn)(vmath::vec<3>));
	static void spin(float xrot, float yrot);
	static void zoom(float z);
	static bool test(vmath::vec<3> dir, vmath::vec<3>* ro_inout, vol::vol_nfo* vol_nfo_out);
	static bool cell_step(vmath::vec<3> dir, vmath::vec<3>* ro_inout, vmath::vec<3> uvw_in, vmath::vec<3, int32_t>* uvw_i_inout, vmath::vec<3>* n_out, bool primary_ray);
};