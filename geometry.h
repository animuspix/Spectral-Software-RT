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
        uint8_t* cell_occupancy; // Range here from 0-255 allows fuzzy stochastic intersections for blurry edges on smooth isosurfaces
        struct vol_nfo
        {
            materials::instance mat;
            math::vec<3> scale; // Bounding-box scale on x/y/z
            math::vec<3> pos; // World-space position
            math::vec<4> orientation; // World-space orientation, specified as a quaternion
        };
        vol_nfo metadata;
        static constexpr uint32_t footprint = res + sizeof(vol_nfo);
    };
    static vol* volumes; // Eventually using a dynamic array for this, adding an actual allocator to my memory system first for better scalability
    static constexpr uint32_t numVolumes = 1;
    static constexpr uint32_t geometry_footprint = numVolumes * (sizeof(vol::vol_nfo) + vol::res);
    static void init();
    static bool test(math::vec<3> dir, math::vec<3>* ro_inout, geometry::vol::vol_nfo** vol_nfo_out);
};

