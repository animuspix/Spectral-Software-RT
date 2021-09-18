#pragma once

#include "math.h"
#include "materials.h"
#include "mem.h"
#include "tmp_vec.h"

class geometry
{
public:
    struct vol
    {
        static constexpr uint32_t width = 1024; // All volumes are 1024 * 1024 * 1024
        static constexpr uint32_t slice_area = width * width;
        static constexpr uint32_t res = slice_area * width;
        static constexpr float cell_size = 1.0f / width;
        struct transform_nfo
        {
            math::vec<3> scale; // Bounding-box scale on x/y/z
            math::vec<3> pos; // World-space position
            math::vec<4> orientation; // World-space orientation, specified as a quaternion
        };

        // Volume metadata (material, transform information)
        struct vol_nfo
        {
            materials::instance mat;
            transform_nfo transf;
        };
        vol_nfo metadata;
        uint64_t cell_states[res / 64]; // One bit/cell; order is left-right/front-back/top-bottom
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
                                        // To test:
                                        // - Evaluate cell & 0b00000000; if zero, step two cells
                                        // - If not zero, test the bit at the current intersection and branch (either step one cell forward, or evaluate the current material)
        static constexpr uint32_t footprint = res + sizeof(vol_nfo);
        static uint32_t index_solver(math::vec<3> uvw_floored, uint64_t* out_bit_ndx_mask) // Returns chunk index directly, bitmask to select the cell within the chunk in [out_bit_ndx_mask]
        {
            uint8_t bit_ndx = static_cast<uint8_t>((floor(fmodf(uvw_floored.z(), 4.0f)) * 4) + // Z-axis, snapped into chunk space
                                                   (floor(fmodf(uvw_floored.y(), 4.0f)) * 16) + // Y-axis, snapped into chunk space
                                                   floor(fmodf(uvw_floored.x(), 4.0f))); // X-axis, snapped into chunk space
            *out_bit_ndx_mask = 1ull << bit_ndx;
            return static_cast<uint32_t>((uvw_floored.x() / 4) + // Local scanline offset
                                        (uvw_floored.y() * width / 4) + // Local slice offset
                                        (uvw_floored.z() * slice_area / 16)); // Volume offset;
        }
        static math::vec<3> uvw_solver(uint32_t cell_ndx) // Designed for mapping [0...res] loop iterations into positions to simplify maths operations; not designed to work with bitfields
                                                          // and chunk indices as input
        {
            uint32_t single_page_ndx = cell_ndx % slice_area;
            return math::vec<3>(static_cast<float>(single_page_ndx % width),
                                static_cast<float>(single_page_ndx / width),
                                static_cast<float>(cell_ndx / slice_area));
        }

        // Helper enum & functions to quickly parse chunk/cell data for intersection tests + cell updates
        enum class CELL_STATUS
        {
            OCCUPIED,
            EMPTY
        };
        void set_cell_state(CELL_STATUS status, math::vec<3> uvw_floored)
        {
            uint64_t bit_selector = 0x0;
            uint32_t chunk_ndx = index_solver(uvw_floored, &bit_selector);
            uint64_t& s = cell_states[chunk_ndx];
            if (status == CELL_STATUS::OCCUPIED) s |= bit_selector;
            else { s &= ~bit_selector; }
        }
        CELL_STATUS test_cell_state(math::vec<3> uvw_floored)
        {
            uint64_t bit_selector = 0x0;
            uint32_t chunk_ndx = index_solver(uvw_floored, &bit_selector);
            uint64_t s = cell_states[chunk_ndx];
            if (s > 0)
            {
                return (s & bit_selector) > 0 ? CELL_STATUS::OCCUPIED :
                                                CELL_STATUS::EMPTY;
            }
            else
            {
                // Zero state, no voxels are set, return immediately
                return CELL_STATUS::EMPTY;
            }
        }
    };
    static vol* volume;
    static void init();

    // Test the bounding geometry for the volume grid
    // Used to quickly mask out rays that immediately hit the sky or an external light source
    static bool test(math::vec<3> dir, math::vec<3>* ro_inout, geometry::vol::vol_nfo* vol_nfo_out);

    // Test for intersections with individual cells within the grid
    // Used for traversal within the grid, before shading occurs in each bounce/ray-step; the API takes a source cell + the bounding volume's transformation
    // info and resolves which of the nearby cells the input ray will intersect next
    // Since our intersection test here is basically testing each plane in order and selecting the one with the minimum distance, I decided it made sense to
    // generate a normal as well - the numerical approximation I had before "worked", but really unreliably, and this way should hopefully be simpler + easier
    // to debug
    // Should create a generic box intersector with parameters for cell/world-space intersections, and call it from this + the main geometry test, instead of
    // having duplicated code here
    // No transform data for this version, since we're working in voxel space and converting back to worldspace afterwards
    // Preferred to straightforward volume marching because of the potential to land inside a cell and "bounce" forever,
    // which this method avoids (irl light flows instead of bouncing, but I do still want to have surface approximations
    // like diffuse/spec surfaces instead of handling everything with subsurface scattering/absorption)
    static bool test_cell_intersection(math::vec<3> dir, math::vec<3>* ro_inout, math::vec<3> src_cell_uvw, math::vec<3>* n_out);
};
