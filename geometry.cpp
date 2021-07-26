#include "geometry.h"
#include "mem.h"
#include <ranges>

geometry::vol* geometry::volume;
void geometry::init()
{
    // Allocate volume memory
    volume = mem::allocate_tracing<vol>(sizeof(vol)); // Generalized volume info
    accelStructure->init(); // Acceleration-structure initializers wrap allocator calls for the volume grid + the octree itself

    // Temporary procedural geometry + first-pass SDF solver
    // Need to update integrator code to utilize these outputs and test [cell_occupancy] for per-voxel intersection state
    // (and also test this code since I wrote it all at once and don't really know if it works or not)
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Temporary procedural geometry - eventually this will be loaded from disk
    for (uint32_t i : countRange(0u, vol::res))
    {
        math::vec<3> uvw = vol::uvw_solver(i);
        if (math::eps_equality(uvw.z(), std::sin(uvw.x())) &&
            math::eps_equality(uvw.z(), std::cos(uvw.y()))) // Regular waves on the XZ plane
        {
            accelStructure->update(i, true); // Inside geometry, set the corresponding bit ^_^
        }
    }

    // Update transform metadata; eventually this should be loaded from disk
    volume->metadata.pos = math::vec<3>(0.0f, 0.0f, 20.0f);
    volume->metadata.orientation = math::vec<4>(0.0f, 0.0f, 0.0f, 1.0f);
    volume->metadata.scale = math::vec<3>(4, 4, 4); // Boring regular unit scale

    // Update material metadata; eventually this should be loaded from disk
    materials::instance& boxMat = volume->metadata.mat;
    boxMat.material_type = material_labels::DIFFUSE;
    boxMat.roughness = 0.2f;
    boxMat.spectral_ior = math::fn<4, const float>(spectra::placeholder_spd);
    boxMat.spectral_response = math::fn<4, const float>(spectra::placeholder_spd);
}

bool geometry::test(math::vec<3> dir, math::vec<3>* ro_inout, geometry::vol::vol_nfo** vol_nfo_out)
{
    // Trashy loop disregards depth and visibility; future versions will perform octree traversal here too
    // Load the current object
    vol::vol_nfo* vol_meta = &volume->metadata;

    // Import object properties
    const math::vec<3> extents = vol_meta->scale * 0.5f; // Extents from object origin
    const math::vec<3> pos = vol_meta->pos;
    const math::vec<4> orientation = vol_meta->orientation;

    // Intersection math adapted from
    // https://www.shadertoy.com/view/ltKyzm, itself adapted from the Scratchapixel tutorial here:
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection

    // Synthesize box boundaries
    const math::vec<3> boundsMin = (pos - extents);
    const math::vec<3> boundsMax = (pos + extents);

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
                        plane_dists[0].z() < sT.y() && sT.x() < plane_dists[1].z()) && (sT.e[0] > 0); // Extend intersection test to ignore intersections behind the current ray  (where the direction
                                                                                                      // to the intersection point is the reverse of the current ray direction)

    // Write out intersection position + shared object info for successful intersections, then early-out
    // (need to sort candidates by depth here)
    if (isect)
    {
        *ro_inout = *ro_inout + (dir * sT.e[0]);
        *vol_nfo_out = vol_meta;
        return true;
    }
    return false;
}
