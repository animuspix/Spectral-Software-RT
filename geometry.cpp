#include "geometry.h"
#include "mem.h"
#include <ranges>

geometry::vol* geometry::volumes;
void geometry::init()
{
    // Allocate triangle memory
    volumes = mem::allocate_tracing<vol>(geometry_footprint);

    // Just one triangle for now, equilateral and ~five units from the camera
    // (import code will go here in future - going to use this tool as the editor too, not super enamoured of triangle modelling or even curve stuff in blender/wherever)
    volumes[0].metadata.pos = math::vec<3>(0.0f, 0.0f, 20.0f);
    volumes[0].metadata.orientation = math::vec<4>(0.0f, 0.0f, 0.0f, 1.0f);
    volumes[0].metadata.scale = math::vec<3>(4, 4, 4); // Boring regular unit scale
    materials::instance& boxMat = volumes[0].metadata.mat;
    boxMat.material_type = material_labels::DIFFUSE;
    boxMat.roughness = 0.2f;
    boxMat.spectral_ior = math::fn<4, const float>(spectra::placeholder_spd);
    boxMat.spectral_response = math::fn<4, const float>(spectra::placeholder_spd);
}

bool geometry::test(math::vec<3> dir, math::vec<3>* ro_inout, geometry::vol::vol_nfo** vol_nfo_out)
{
    // Trashy loop disregards depth and visibility; future versions will perform octree traversal here too
    for (uint32_t i = 0; i < numVolumes; i++)
    {
        // Load the current object
        vol cur_vol = volumes[i];

        // Import object properties
        const math::vec<3> scale = cur_vol.metadata.scale;
        const math::vec<3> pos = cur_vol.metadata.pos;
        const math::vec<4> orientation = cur_vol.metadata.orientation;

        // Intersection math adapted from
        // https://www.shadertoy.com/view/ltKyzm, itself adapted from the Scratchapixel tutotiral here:
        // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection

        // Synthesize box boundaries
        scale *= 0.5f;
        const math::vec<3> boundsMin = (pos - scale);
        const math::vec<3> boundsMax = (pos + scale);

        // Evaluate per-axis distances to each plane in the box
        // Not super sure why these divisions happen?
        // Should probably reread math for all this in general
        math::vec<3> plane_dists[2] =
        {
            (boundsMin - *ro_inout) / dir,
            (boundsMax - *ro_inout) / dir
        };

        // Keep near distances in [0], far distances in [1]
        const math::vec<3> dist0 = plane_dists[0];
        const math::vec<3> dist1 = plane_dists[1];
        plane_dists[0].e[0] = std::min(dist0.x(), dist1.x());
        plane_dists[0].e[1] = std::min(dist0.y(), dist1.y());
        plane_dists[0].e[2] = std::min(dist0.z(), dist1.z());
        plane_dists[1].e[0] = std::max(dist0.x(), dist1.x());
        plane_dists[1].e[1] = std::max(dist0.y(), dist1.y());
        plane_dists[1].e[2] = std::max(dist0.z(), dist1.z());

        // Evaluate scalar min/max distances for the given ray
        math::vec<2> sT = math::vec<2>(std::max(std::max(plane_dists[0].x(), plane_dists[0].y()), plane_dists[0].z()),
            std::min(std::min(plane_dists[1].x(), plane_dists[1].y()), plane_dists[1].z()));
        sT = math::vec<2>(std::min(sT.x(), sT.y()), std::max(sT.x(), sT.y())); // Keep near distance in [x], far distance in [y]

        // Resolve intersection status
        const bool isect = (plane_dists[0].x() < plane_dists[1].y() && plane_dists[0].y() < plane_dists[1].x() &&
            plane_dists[0].z() < sT.y() && sT.x() < plane_dists[1].z());

        // Write out intersection position + shared object info for successful intersections, then early-out
        // (need to sort candidates by depth here)
        if (isect)
        {
            *ro_inout = *ro_inout + (dir * sT.e[0]);
            *vol_nfo_out = &volumes[i].metadata;
            return true;
        }
    }
    return false;
}
