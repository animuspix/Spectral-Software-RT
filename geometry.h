#pragma once

#include "math.h"
#include "materials.h"

class geometry
{
public:
    struct vol
    {
        static constexpr uint32_t width = 1024; // All volumes are 1024 * 1024 * 1024
        static constexpr uint32_t res = 1024 * 1024 * 1024;
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
                   (uvw_floored.y() * geometry::vol::width) + // Local slice offset
                   (uvw_floored.z() * geometry::vol::width * geometry::vol::width); // Volume offset
        }
    };
    static vol* volumes; // Eventually using a dynamic array for this, adding an actual allocator to my memory system first for better scalability
    static constexpr uint32_t numVolumes = 1;
    static constexpr uint32_t geometry_footprint = numVolumes * (sizeof(vol::vol_nfo) + vol::res);
    static void init();
    static bool test(math::vec<3> dir, math::vec<3>* ro_inout, geometry::vol::vol_nfo** vol_nfo_out);
};

