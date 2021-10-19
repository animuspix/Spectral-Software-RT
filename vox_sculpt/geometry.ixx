export module geometry;

#pragma once

import vmath;
import materials;
import mem;
import platform;
import spectra;
import vox_ints;

namespace geometry
{
    export struct vol
    {
        static constexpr u32 width = 1024; // All volumes are 1024 * 1024 * 1024
        static constexpr u32 slice_area = width * width;
        static constexpr u32 res = slice_area * width;
        static constexpr float cell_size = 1.0f / width;
        static constexpr u32 max_cell_ndx_per_axis = width - 1;
        struct transform_nfo
        {
            vmath::vec<3> scale; // Bounding-box scale on x/y/z
            vmath::vec<3> pos; // World-space position
            vmath::vec<4> orientation; // World-space orientation, specified as a quaternion
        };

        // Volume metadata (material, transform information)
        struct vol_nfo
        {
            materials::instance mat;
            transform_nfo transf;
        };
        vol_nfo metadata;
        u64 cell_states[res / 64]; // One bit/cell; order is left-right/front-back/top-bottom
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
        static constexpr u32 footprint = res + sizeof(vol_nfo);
        static u32 index_solver(vmath::vec<3> uvw_floored, u64* out_bit_ndx_mask) // Returns chunk index directly, bitmask to select the cell within the chunk in [out_bit_ndx_mask]
        {
            u8 bit_ndx = static_cast<u8>((floor(fmodf(uvw_floored.z(), 4.0f)) * 4) + // Z-axis, snapped into chunk space
                                         (floor(fmodf(uvw_floored.y(), 4.0f)) * 16) + // Y-axis, snapped into chunk space
                                         floor(fmodf(uvw_floored.x(), 4.0f))); // X-axis, snapped into chunk space
            *out_bit_ndx_mask = 1ull << bit_ndx;
            return static_cast<u32>((uvw_floored.x() / 4) + // Local scanline offset
                                    (uvw_floored.y() * width / 4) + // Local slice offset
                                    (uvw_floored.z() * slice_area / 16)); // Volume offset;
        }
        static vmath::vec<3> uvw_solver(u32 cell_ndx) // Designed for mapping [0...res] loop iterations into positions to simplify vmaths operations; not designed to work with bitfields
                                                      // and chunk indices as input
        {
            u32 single_page_ndx = cell_ndx % slice_area;
            return vmath::vec<3>(static_cast<float>(single_page_ndx % width),
                                 static_cast<float>(single_page_ndx / width),
                                 static_cast<float>(cell_ndx / slice_area));
        }

        // Helper enum & functions to quickly parse chunk/cell data for intersection tests + cell updates
        enum class CELL_STATUS
        {
            OCCUPIED,
            EMPTY
        };
        void set_cell_state(CELL_STATUS status, vmath::vec<3> uvw_floored)
        {
            u64 bit_selector = 0x0;
            u32 chunk_ndx = index_solver(uvw_floored, &bit_selector);
            u64& s = cell_states[chunk_ndx];
            if (status == CELL_STATUS::OCCUPIED) s |= bit_selector;
            else { s &= ~bit_selector; }
        }
        CELL_STATUS test_cell_state(vmath::vec<3> uvw_floored)
        {
            u64 bit_selector = 0x0;
            u32 chunk_ndx = index_solver(uvw_floored, &bit_selector);
            u64 s = cell_states[chunk_ndx];
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
    export vol* volume;
    export void init()
    {
        // Allocate volume memory
        volume = mem::allocate_tracing<vol>(vol::footprint); // Generalized volume info
        platform::osClearMem(volume->cell_states, vol::res / 8);

        // Temporary procedural geometry - eventually this will be loaded from disk
        // Huge speedup using parallel_for here; when we begin loading from disk we should try to maintain parallelism using custom threads
        // (nine threads doing four slices at a time each, using a specialized accessor to fill/write one chunk/64 bits in each step)
        for (u32 i = 0; i < vol::res; i++)
        {
            vmath::vec<3> uvw = vol::uvw_solver(i);
            //#define GEOM_DEMO_WAVES
            //#define GEOM_DEMO_STRIPES
#define GEOM_DEMO_SPHERE
//#define GEOM_DEMO_CIRCLE
#ifdef GEOM_DEMO_STRIPES
                uvw = vmath::floor(uvw);
                vol::CELL_STATUS stat_setting = ((u32)(uvw.x() / 64) % 2) ? geometry::vol::CELL_STATUS::OCCUPIED :
                    geometry::vol::CELL_STATUS::EMPTY;
                volume->set_cell_state(stat_setting, uvw);
#elif defined(GEOM_DEMO_WAVES)
                constexpr float num_waves = 8.0f;
                constexpr float stretch = vol::width / num_waves / vmath::pi;
                constexpr float ampli = stretch;
                if (uvw.y() < (((std::sin(uvw.x() / stretch)) * ampli) + 512))
                {
                    volume->set_cell_state(geometry::vol::CELL_STATUS::OCCUPIED, vmath::floor(uvw));
                }
                else
                {
                    volume->set_cell_state(geometry::vol::CELL_STATUS::EMPTY, vmath::floor(uvw));
                }
#elif defined(GEOM_DEMO_CIRCLE)
                constexpr float r = 512.0f;
                const vmath::vec<2> circOrigin(512, 512);
                if ((circOrigin - uvw.xy()).magnitude() < r)
                {
                    volume->set_cell_state(geometry::vol::CELL_STATUS::OCCUPIED, vmath::floor(uvw));
                }
                else
                {
                    volume->set_cell_state(geometry::vol::CELL_STATUS::EMPTY, vmath::floor(uvw));
                }
#elif defined(GEOM_DEMO_SPHERE)
                constexpr float r = 512.0f;
                const vmath::vec<3> circOrigin(512, 512, 512);
                if ((circOrigin - uvw).magnitude() < r)
                {
                    volume->set_cell_state(geometry::vol::CELL_STATUS::OCCUPIED, vmath::vfloor(uvw));
                }
                else
                {
                    volume->set_cell_state(geometry::vol::CELL_STATUS::EMPTY, vmath::vfloor(uvw));
                }
#elif defined(GEOM_DEMO_SOLID)
                volume->set_cell_state(geometry::vol::CELL_STATUS::OCCUPIED, uvw);
#endif
            };

        // Black-box test box set/unset code on every cell, using the stripey test pattern shown above
    //#define VALIDATE_CELL_OPS
#ifdef VALIDATE_CELL_OPS
        ZeroMemory(volume->cell_states, vol::res / 8);
        for (u32 i : countRange(0u, vol::res))
        {
            vmath::vec<3> uvw = vol::uvw_solver(i);
            uvw = vmath::floor(uvw);
            vol::CELL_STATUS stat_setting = ((u32)(uvw.x() / 64) % 2) ? geometry::vol::CELL_STATUS::OCCUPIED :
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

        // Possible debugging helper for geometry here; build in a .png exporter, write out cells on a certain slice to black or white depending on activation status
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Update transform metadata; eventually this should be loaded from disk
        volume->metadata.transf.pos = vmath::vec<3>(0.0f, 0.0f, 20.0f);
        volume->metadata.transf.orientation = vmath::vec<4>(0.0f, 0.0f, 0.0f, 1.0f);
        volume->metadata.transf.scale = vmath::vec<3>(4, 4, 4); // Boring regular unit scale

        // Update material metadata; eventually this should be loaded from disk
        materials::instance& boxMat = volume->metadata.mat;
        boxMat.material_type = material_labels::DIFFUSE;
        boxMat.roughness = 0.2f;
        boxMat.spectral_ior = vmath::fn<4, const float>(spectra::placeholder_spd);
        boxMat.spectral_response = vmath::fn<4, const float>(spectra::placeholder_spd);
    }

    // Test the bounding geometry for the volume grid
    // Used to quickly mask out rays that immediately hit the sky or an external light source
    export bool test(vmath::vec<3> dir, vmath::vec<3>* ro_inout, geometry::vol::vol_nfo* vol_nfo_out)
    {
        // Import object properties
        const vmath::vec<3> extents = volume->metadata.transf.scale * 0.5f; // Extents from object origin
        const vmath::vec<3> pos = volume->metadata.transf.pos;
        const vmath::vec<4> orientation = volume->metadata.transf.orientation;

        // Intersection vmath adapted from
        // https://www.shadertoy.com/view/ltKyzm, itself adapted from the Scratchapixel tutorial here:
        // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection

        // Synthesize box boundaries
        const vmath::vec<3> boundsMin = (pos - extents);
        const vmath::vec<3> boundsMax = (pos + extents);

        // Evaluate per-axis distances to each plane in the box
        // Not super sure why these divisions happen?
        // Should probably reread vmath for all this in general
        vmath::vec<3> plane_dists[2] =
        {
            (boundsMin - *ro_inout) / dir,
            (boundsMax - *ro_inout) / dir
        };

        // Keep near distances in [0], far distances in [1]
        const vmath::vec<3> dist0 = plane_dists[0];
        const vmath::vec<3> dist1 = plane_dists[1];
        plane_dists[0].e[0] = vmath::fmin(dist0.x(), dist1.x());
        plane_dists[0].e[1] = vmath::fmin(dist0.y(), dist1.y());
        plane_dists[0].e[2] = vmath::fmin(dist0.z(), dist1.z());
        plane_dists[1].e[0] = vmath::fmax(dist0.x(), dist1.x());
        plane_dists[1].e[1] = vmath::fmax(dist0.y(), dist1.y());
        plane_dists[1].e[2] = vmath::fmax(dist0.z(), dist1.z());

        // Evaluate scalar min/max distances for the given ray
        vmath::vec<2> sT = vmath::vec<2>(vmath::fmax(vmath::fmax(plane_dists[0].x(), plane_dists[0].y()), plane_dists[0].z()),
                                         vmath::fmin(vmath::fmin(plane_dists[1].x(), plane_dists[1].y()), plane_dists[1].z()));
        sT = vmath::vec<2>(vmath::fmin(sT.x(), sT.y()), vmath::fmax(sT.x(), sT.y())); // Keep near distance in [x], far distance in [y]

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
    struct intersection_plane
    {
        float dist;
        vmath::vec<3> n;
        intersection_plane(float _dist, vmath::vec<3> _n) : dist(_dist), n(_n) {}
        static intersection_plane min(intersection_plane lhs, intersection_plane rhs)
        {
            return lhs.dist > rhs.dist ? rhs : lhs;
        }
        static intersection_plane max(intersection_plane lhs, intersection_plane rhs)
        {
            return lhs.dist > rhs.dist ? lhs : rhs;
        }
    };
    export bool test_cell_intersection(vmath::vec<3> dir, vmath::vec<3>* ro_inout, vmath::vec<3> src_cell_uvw, vmath::vec<3>* n_out)
    {
        // Cache fixed transform values
        const vmath::vec<3> extents = vmath::vec<3>(0.5f, 0.5f, 0.5f); // Extents from object origin

        // Walk through each neighbour of the given cell
        for (i32 i = -1; i <= 1; i++)
        {
            for (i32 j = -1; j <= 1; j++) // Grr? Not totally certain how to optimize this, I don't think naive loop unrolling will be enough
            {
                for (i32 k = -1; k <= 1; k++)
                {
                    // Avoid testing our home cell
                    if (i == 0 && j == 0 && k == 0) continue;

                    // Otherwise resolve the target cell
                    vmath::vec<3> cell_offs = vmath::vec<3>((float)i, (float)j, (float)k);
                    vmath::vec<3> target_cell = src_cell_uvw + cell_offs;

                    // Avoid testing any cells in the opposite direction the ray (since we don't care about them anyways)
                    const float cell_vis = dir.dot(cell_offs.normalized());
                    if (cell_vis <= 0) continue;

                    // Avoid testing any cells outside the defined part of the volume
                    if (vmath::anyGreater(target_cell, vol::max_cell_ndx_per_axis) || vmath::anyLesser(target_cell, 0)) continue;

                    // Compute an intersection test on the current neightbour; return immediately when an intersection is found
                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

                    // Intersection vmath adapted from
                    // https://www.shadertoy.com/view/ltKyzm, itself adapted from the Scratchapixel tutorial here:
                    // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection

                    // Synthesize box boundaries
                    const vmath::vec<3> boundsMin = (target_cell - extents);
                    const vmath::vec<3> boundsMax = (target_cell + extents);

                    // Evaluate per-axis distances to each plane in the box
                    // Not super sure why these divisions happen?
                    // Should probably reread vmath for all this in general
                    intersection_plane plane_dists[6] =
                    {
                        intersection_plane((boundsMin.x() - src_cell_uvw.x()) / dir.x(), vmath::vec<3>(-1,0,0)),
                        intersection_plane((boundsMin.y() - src_cell_uvw.y()) / dir.y(), vmath::vec<3>(0,-1,0)),
                        intersection_plane((boundsMin.z() - src_cell_uvw.z()) / dir.z(), vmath::vec<3>(0,0,-1)),
                        ///////////////////////////////////////////////////////////////////////////////////////
                        intersection_plane((boundsMax.x() - src_cell_uvw.x()) / dir.x(), vmath::vec<3>(1,0,0)),
                        intersection_plane((boundsMax.y() - src_cell_uvw.y()) / dir.y(), vmath::vec<3>(0,1,0)),
                        intersection_plane((boundsMax.z() - src_cell_uvw.z()) / dir.z(), vmath::vec<3>(0,0,1)),
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
                    intersection_plane sT[2] = { intersection_plane::max(intersection_plane::max(plane_dists_sorted[0], plane_dists_sorted[1]), plane_dists_sorted[2]),
                                                 intersection_plane::min(intersection_plane::min(plane_dists_sorted[3], plane_dists_sorted[4]), plane_dists_sorted[5]) };
                    intersection_plane sT_min = intersection_plane::min(sT[0], sT[1]); // Keep near/far distances correctly ordered
                    intersection_plane sT_max = intersection_plane::max(sT[0], sT[1]);
                    sT[0] = sT_min;
                    sT[1] = sT_max;

                    // Resolve intersection status
                    const bool isect = (plane_dists_sorted[0].dist < plane_dists_sorted[4].dist&& plane_dists_sorted[1].dist < plane_dists_sorted[3].dist&&
                                        plane_dists_sorted[2].dist < sT[1].dist&& sT[0].dist < plane_dists_sorted[5].dist) && (sT[0].dist > 0); // Extend intersection test to ignore intersections behind the current ray (where the direction
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
};
