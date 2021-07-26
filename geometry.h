#pragma once

#include "math.h"
#include "materials.h"

class geometry
{
public:
    struct vol
    {
        static constexpr uint32_t width = 1024; // All volumes are 1024 * 1024 * 1024
        static constexpr uint32_t slice_area = width * width;
        static constexpr uint32_t res = slice_area * width;
        static constexpr uint8_t max_cell_dist = 127;
        static constexpr uint8_t occupancy_bits = 1; // No need for any more bits than this, but easier to read than [dist << 1]
        uint8_t* cell_occupancy; // First seven bits are rough SDF distance (up to 127) for fast SDF marching, eighth bit gives binary occupancy (whether a cell is/isn't vacuum)
        struct vol_nfo
        {
            materials::instance mat;
            math::vec<3> scale; // Bounding-box scale on x/y/z
            math::vec<3> pos; // World-space position
            math::vec<4> orientation; // World-space orientation, specified as a quaternion
        };
        vol_nfo metadata;
        static constexpr uint32_t footprint = res + sizeof(vol_nfo);
        static uint32_t index_solver(math::vec<3> uvw_floored)
        {
            return uvw_floored.x() + // Local scanline offset
                   (uvw_floored.y() * width) + // Local slice offset
                   (uvw_floored.z() * slice_area); // Volume offset
        }
        static math::vec<3> uvw_solver(uint32_t cell_ndx)
        {
            return math::vec<3>(cell_ndx % width,
                                cell_ndx / width,
                                cell_ndx / slice_area);
        }
        static bool cell_validity(math::vec<3> uvw) // Test whether the given cell coordinate lies inside the grid; code frequently
                                                    // needs to find whether a valid coordinate + an offset is occluded (for e.g. testing candidate voxel normals,
                                                    // building our volume SDFs from a search kernel), but the offsetted values need to be range-tested for safety (otherwise
                                                    // we'll give ourselves out-of-bounds errors) - collecting those range-checks here should avoid duplication and help keep
                                                    // things reliable
        {
            return uvw.x() < width && uvw.x() > 0 &&
                   uvw.x() < width && uvw.x() > 0 &&
                   uvw.x() < width && uvw.x() > 0;
        }
    };
    static vol* volumes; // Eventually using a dynamic array for this, adding an actual allocator to my memory system first for better scalability
                         // ...probably never going to support multiple volumes (just making a sculpting tool anyway), should treat this as one single volume instead
    static constexpr uint32_t numVolumes = 1;
    static constexpr uint32_t geometry_footprint = numVolumes * (sizeof(vol::vol_nfo) + vol::res);
    static void init();
    static bool test(math::vec<3> dir, math::vec<3>* ro_inout, geometry::vol::vol_nfo** vol_nfo_out);
};

