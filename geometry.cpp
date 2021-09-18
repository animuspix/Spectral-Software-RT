#include "geometry.h"
#include "mem.h"
#include <ppl.h>

geometry::vol* geometry::volume;
void geometry::init()
{
    // Allocate volume memory
    volume = mem::allocate_tracing<vol>(vol::footprint); // Generalized volume info
    ZeroMemory(volume->cell_states, vol::res / 8);

    // Temporary procedural geometry - eventually this will be loaded from disk
    // Huge speedup using parallel_for here; when we begin loading from disk we should try to maintain parallelism using custom threads
    // (ten threads doing four slices at a time each, using a specialized accessor to fill/write one chunk/64 bits in each step)
    concurrency::parallel_for((uint64_t)0, (uint64_t)vol::res, [&](uint64_t i)
    //for (uint32_t i : countRange(0u, vol::res))
    {
        math::vec<3> uvw = vol::uvw_solver(i);
//#define GEOM_DEMO_WAVES
//#define GEOM_DEMO_STRIPES
#define GEOM_DEMO_SPHERE // Leaving spheres and other 3D geometry until I get around to writing a better algorithm for marching within the volume grid
//#define GEOM_DEMO_CIRCLE
#ifdef GEOM_DEMO_STRIPES
        uvw = math::floor(uvw);
        vol::CELL_STATUS stat_setting = ((uint32_t)(uvw.x() / 64) % 2) ? geometry::vol::CELL_STATUS::OCCUPIED :
                                                                         geometry::vol::CELL_STATUS::EMPTY;
        volume->set_cell_state(stat_setting, uvw);
#elif defined(GEOM_DEMO_WAVES)
        constexpr float num_waves = 8.0f;
        constexpr float stretch = vol::width / num_waves / math::pi;
        constexpr float ampli = stretch;
        if (uvw.y() < (((std::sin(uvw.x() / stretch)) * ampli) + 512))
        {
            volume->set_cell_state(geometry::vol::CELL_STATUS::OCCUPIED, math::floor(uvw));
        }
        else
        {
            volume->set_cell_state(geometry::vol::CELL_STATUS::EMPTY, math::floor(uvw));
        }
#elif defined(GEOM_DEMO_CIRCLE)
        constexpr float r = 512.0f;
        const math::vec<2> circOrigin(512, 512);
        if ((circOrigin - uvw.xy()).magnitude() < r)
        {
            volume->set_cell_state(geometry::vol::CELL_STATUS::OCCUPIED, math::floor(uvw));
        }
        else
        {
            volume->set_cell_state(geometry::vol::CELL_STATUS::EMPTY, math::floor(uvw));
        }
#elif defined(GEOM_DEMO_SPHERE)
        constexpr float r = 512.0f;
        const math::vec<3> circOrigin(512, 512, 512);
        if ((circOrigin - uvw).magnitude() < r)
        {
            volume->set_cell_state(geometry::vol::CELL_STATUS::OCCUPIED, math::floor(uvw));
        }
        else
        {
            volume->set_cell_state(geometry::vol::CELL_STATUS::EMPTY, math::floor(uvw));
        }
#elif defined(GEOM_DEMO_SOLID)
        volume->set_cell_state(geometry::vol::CELL_STATUS::OCCUPIED, uvw);
#endif
    });

    // Black-box test box set/unset code on every cell, using the stripey test pattern shown above
//#define VALIDATE_CELL_OPS
#ifdef VALIDATE_CELL_OPS
    ZeroMemory(volume->cell_states, vol::res / 8);
    for (uint32_t i : countRange(0u, vol::res))
    {
        math::vec<3> uvw = vol::uvw_solver(i);
        uvw = math::floor(uvw);
        vol::CELL_STATUS stat_setting = ((uint32_t)(uvw.x() / 64) % 2) ? geometry::vol::CELL_STATUS::OCCUPIED :
                                                                         geometry::vol::CELL_STATUS::EMPTY;
        volume->set_cell_state(stat_setting, uvw);
        vol::CELL_STATUS status = volume->test_cell_state(uvw);
        if (status != stat_setting)
        {
            DebugBreak();
        }
        //volume->set_cell(geometry::vol::CELL_STATUS::OCCUPIED, uvw);
    }//);
#endif

    // Update transform metadata; eventually this should be loaded from disk
    volume->metadata.transf.pos = math::vec<3>(0.0f, 0.0f, 20.0f);
    volume->metadata.transf.orientation = math::vec<4>(0.0f, 0.0f, 0.0f, 1.0f);
    volume->metadata.transf.scale = math::vec<3>(4, 4, 4); // Boring regular unit scale

    // Update material metadata; eventually this should be loaded from disk
    materials::instance& boxMat = volume->metadata.mat;
    boxMat.material_type = material_labels::DIFFUSE;
    boxMat.roughness = 0.2f;
    boxMat.spectral_ior = math::fn<4, const float>(spectra::placeholder_spd);
    boxMat.spectral_response = math::fn<4, const float>(spectra::placeholder_spd);
}

bool geometry::test(math::vec<3> dir, math::vec<3>* ro_inout, geometry::vol::vol_nfo* vol_nfo_out)
{
    // Import object properties
    const math::vec<3> extents = volume->metadata.transf.scale * 0.5f; // Extents from object origin
    const math::vec<3> pos = volume->metadata.transf.pos;
    const math::vec<4> orientation = volume->metadata.transf.orientation;

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
        *vol_nfo_out = volume->metadata;
        return true;
    }
    return false;
}

struct intersection_plane
{
    float dist;
    math::vec<3> n;
    intersection_plane(float _dist, math::vec<3> _n) : dist(_dist), n(_n) {}
    static intersection_plane min(intersection_plane lhs, intersection_plane rhs)
    {
        return lhs.dist > rhs.dist ? rhs : lhs;
    }
    static intersection_plane max(intersection_plane lhs, intersection_plane rhs)
    {
        return lhs.dist > rhs.dist ? lhs : rhs;
    }
};
bool geometry::test_cell_intersection(math::vec<3> dir, math::vec<3>* ro_inout, math::vec<3> src_cell_uvw, math::vec<3>* n_out)
{
    // Cache fixed transform values
    const math::vec<3> extents = math::vec<3>(0.5f, 0.5f, 0.5f); // Extents from object origin

    // Walk through each neighbour of the given cell
    for (int32_t i = -1; i <= 1; i++)
    {
        for (int32_t j = -1; j <= 1; j++) // Grr? Not totally certain how to optimize this, I don't think naive loop unrolling will be enough
        {
            for (int32_t k = -1; k <= 1; k++)
            {
                // Avoid testing our home cell
                if (i == 0 && j == 0 && k == 0) continue;

                // Otherwise resolve the target cell
                math::vec<3> cell_offs = math::vec<3>(i, j, k);
                math::vec<3> target_cell = src_cell_uvw + cell_offs;

                // Avoid testing any cells in the opposite direction the ray (since we don't care about them anyways)
                const float cell_vis = dir.dot(cell_offs.normalized());
                if (cell_vis <= 0) continue;

                // Avoid testing any cells outside the defined part of the volume
                if (math::anyGreater(target_cell, vol::width) || math::anyLesser(target_cell, 0)) continue;

                // Compute an intersection test on the current neightbour; return immediately when an intersection is found
                ///////////////////////////////////////////////////////////////////////////////////////////////////////////

                // Intersection math adapted from
                // https://www.shadertoy.com/view/ltKyzm, itself adapted from the Scratchapixel tutorial here:
                // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection

                // Synthesize box boundaries
                const math::vec<3> boundsMin = (target_cell - extents);
                const math::vec<3> boundsMax = (target_cell + extents);

                // Evaluate per-axis distances to each plane in the box
                // Not super sure why these divisions happen?
                // Should probably reread math for all this in general
                intersection_plane plane_dists[6] =
                {
                    intersection_plane((boundsMin.x() - src_cell_uvw.x()) / dir.x(), math::vec<3>(-1,0,0)),
                    intersection_plane((boundsMin.y() - src_cell_uvw.y()) / dir.y(), math::vec<3>(0,-1,0)),
                    intersection_plane((boundsMin.z() - src_cell_uvw.z()) / dir.z(), math::vec<3>(0,0,-1)),
                    ///////////////////////////////////////////////////////////////////////////////////////
                    intersection_plane((boundsMax.x() - src_cell_uvw.x()) / dir.x(), math::vec<3>(1,0,0)),
                    intersection_plane((boundsMax.y() - src_cell_uvw.y()) / dir.y(), math::vec<3>(0,1,0)),
                    intersection_plane((boundsMax.z() - src_cell_uvw.z()) / dir.z(), math::vec<3>(0,0,1)),
                };

                // Keep min/max distances correctly ordered
                intersection_plane plane_dists_sorted[6] =
                {
                    intersection_plane::min(plane_dists[0], plane_dists[3]),
                    intersection_plane::min(plane_dists[1], plane_dists[4]),
                    intersection_plane::min(plane_dists[2], plane_dists[5]),
                    intersection_plane::max(plane_dists[0], plane_dists[3]),
                    intersection_plane::max(plane_dists[1], plane_dists[4]),
                    intersection_plane::max(plane_dists[2], plane_dists[5])
                };

                // Evaluate scalar min/max distances for the given ray
                intersection_plane sT[2] = { intersection_plane::min(intersection_plane::min(plane_dists_sorted[0], plane_dists_sorted[1]), plane_dists_sorted[2]),
                                             intersection_plane::max(intersection_plane::max(plane_dists_sorted[3], plane_dists_sorted[4]), plane_dists_sorted[5]) };
                intersection_plane sT_min = intersection_plane::min(sT[0], sT[1]); // Keep near/far distances correctly ordered
                intersection_plane sT_max = intersection_plane::max(sT[0], sT[1]);
                sT[0] = sT_min;
                sT[1] = sT_max;

                // Resolve intersection status
                const bool isect = (plane_dists_sorted[0].dist < plane_dists_sorted[4].dist && plane_dists_sorted[1].dist < plane_dists_sorted[3].dist &&
                                    plane_dists_sorted[5].dist < sT[1].dist && sT[0].dist < plane_dists_sorted[5].dist) && (sT[0].dist > 0); // Extend intersection test to ignore intersections behind the current ray (where the direction
                                                                                                                                             // to the intersection point is the reverse of the current ray direction)

                // Write out intersection position + shared object info for successful intersections, then early-out
                if (isect)
                {
                    *n_out = sT[0].n;
                    *ro_inout = *ro_inout + (dir * sT[0].dist * vol::cell_size * volume->metadata.transf.scale); // Update world-space position, after converting displacement from voxel space
                                                                                                                 // (scale once for cell size, again for volume scale)
                    return true;
                }
            }
        }
    }

    // Only possible way to get here is by failing to find an intersection above
    return false;
}
