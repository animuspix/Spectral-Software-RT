#include "geometry.h"
#include "mem.h"
#include <ranges>

geometry::vol* geometry::volumes;
void geometry::init()
{
    // Allocate triangle memory
    volumes = mem::allocate_tracing<vol>(geometry_footprint);

    // Just one homogenous box for now, regular and ~20 units from the camera
    // (import code will go here in future - going to use this tool as the editor too, not super enamoured of triangle modelling or even curve stuff in blender/wherever)
    volumes[0].cell_occupancy = mem::allocate_tracing<uint8_t>(vol::res);

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
            volumes[0].cell_occupancy[i] |= 0x1; // Inside geometry, set the occupancy bit :D
        }
    }

    // Geometric SDF solver, abstracted away from any specific function so we can keep using it once we get previs/editing working
    uint8_t* candidate_distances = mem::allocate_tracing<uint8_t>(vol::max_cell_dist * vol::max_cell_dist); // Allocate temporary per-kernel candidate distance array
    uint16_t candidate_count = 0;
    for (uint32_t i : countRange(0u, vol::res)) // gross on^5+ complexity, but easy to visualize
                                                // Expecting to thread this eventally (keeping it simples, one thread/cell spaced max_cell_dist units apart to avoid contention)
    {
        // For each voxel, test an expanding kernel and early-out at the first intersection
        // This is separable! but my memory for convolutions is fuzzy so I'll optimize that later :D
        // Since this is likely to run exactly once, and separate to any tracing work, its probably viable to multi-thread it and test several kernels in parallel
        // That might have greater yield and be simpler to write than converting to separated passes
        // Eventually this transform will run as a baking pass over WIP volumes before final renders (occupancy data can be edited live & used for previs)
        // How does this work? For each voxel there is a minimum-radius circle/quadratic that intersects the volume; our probe is designed to find the smallest one by assigning
        // distances (radii) to concentric points around each sample, and exiting once the kernel overlaps an isoboundary (=> one of the candidate circles hits the surface). You
        // can imagine each point in the kernel as representing a separate circle in the output.
        // This algorithm was inspired by the explanation here:
        // https://prideout.net/blog/distance_fields/ (although their version is optimized into separable passes)
        int8_t kernRad = 1; // Maximum distance supported by each cell is 127 (giving a kernel/circle/probe width of 255)
        uint8_t occupancy = 0; // Assume cells are empty by default
        bool circle_isect = false; // Has our kernel found an intersection?
        math::vec<3> uvw = geometry::vol::uvw_solver(i);
        while (!circle_isect) // Find candidate distances
        {
            auto cell_test_and_output = [&](math::vec<3> _probe)
            {
                if (vol::cell_validity(_probe)) // Safety test! Want to make sure our indices lie inside the volume grid before we perform any array accesses/occupancy checks
                {
                    if (volumes[0].cell_occupancy[geometry::vol::index_solver(_probe)] & 1) // Raster cell occupancy, ignoring any existing distance data
                    {
                        candidate_distances[candidate_count] = std::max((uint8_t)((_probe - uvw).magnitude()), vol::max_cell_dist); // We've restricted our search space to spaces where
                                                                                                                                    // we know total distance will be <= 128, so we shouldn't
                                                                                                                                    // need to call std::floor(...) here
                        candidate_count += 1;
                        circle_isect = true; // Mark successful intersection, but don't break out - we want to collect every candidate in the current kernel
                    }
                }
            };
            if (kernRad < 74) // Magic number, resolved before build time; the lower bound of positions where every axis is equal and length(p) >= 128
                              // Considering writing a crazy gross compile-time function to generate numerically at build time so I can think less
                              // (naive algorithm might not be too scary, [while((x^2 + y^2 + z^2) <= max_cell_dist_squared) { x++; y++; z++; }])
            {
                math::vec<3> probe = uvw;
                for (int8_t z = -kernRad; z < kernRad; z++) // Slices in the current kernel/circle/quadratic
                {
                    probe.e[2] = uvw.z() + z;
                    for (int8_t y = -kernRad; y < kernRad; y++) // Columns in the current kernel/circle/quadratic
                    {
                        probe.e[1] = uvw.y() + y;
                        for (int8_t x = -kernRad; x < kernRad; x++) // Rows in the current kernel/circle/quadratic
                        {
                            probe.e[0] = uvw.x() + x;
                            cell_test_and_output(probe);
                        }
                    }
                }
                kernRad += 1;
            }
            else
            {
                // We've restricted the search space to points where sqrt(x^2 + y^2 + z^2) < 128 (assuming max_cell_dist == 128)
                // Probably simpler to recompute these bounds as needed than to automate that math (for now)
                // 73/73/73 is the highest value where every axis can be equal and the total length will be less than 128; any further work
                // in the search space needs to test (74/74/73, 73/74/74, 74/73/74) and then only increment each axis complementarily to the
                // others (so x++,y--,y--, x--,y++,z--, and x--,y--,z++)
                // After some (far too much) math this should save ~86% of total area compared to iterating the full kernel - will be interesting
                // to count the number of cells we iterate through and compare to the theoretical volume (~2,288,086 cells)
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                // Useful coordinates
                auto uvw_inflection = uvw + math::vec<3>(kernRad, kernRad, kernRad);
                auto probe_x = math::vec<3>(0, 0, 0);
                auto probe_y = math::vec<3>(0, 0, 0);
                auto probe_z = math::vec<3>(0, 0, 0);

                // Z-tests
                for (int8_t z = kernRad; z < vol::max_cell_dist; z++) // Starting from zero so we don't repeat the inner kernel we computed above
                {
                    for (int8_t x = -kernRad; x < kernRad; x++)
                    {
                        for (int8_t y = -kernRad; y < kernRad; x++)
                        {
                            // Compute positive cone
                            probe_z = uvw_inflection + math::vec<3>(x, y, z);
                            cell_test_and_output(probe_z);

                            // Compute negative cone
                            probe_z = uvw_inflection + math::vec<3>(x, y, -z);
                            cell_test_and_output(probe_z);
                        }
                    }
                }

                // Y-tests
                for (int8_t y = kernRad; y < vol::max_cell_dist; y++)
                {
                    for (int8_t x = -kernRad; x < kernRad; x++)
                    {
                        for (int8_t z = -kernRad; z < kernRad; z++)
                        {
                            // Compute positive cone
                            probe_y = uvw_inflection + math::vec<3>(x, y, z);
                            cell_test_and_output(probe_y);

                            // Compute negative cone
                            probe_y = uvw_inflection + math::vec<3>(x, -y, z);
                            cell_test_and_output(probe_y);
                        }
                    }
                }

                // X-tests
                auto probe_x = uvw;
                for (int8_t x = kernRad; x < vol::max_cell_dist; x++)
                {
                    for (int8_t y = -kernRad; y < kernRad; x++)
                    {
                        for (int8_t z = -kernRad; z < kernRad; z++)
                        {
                            // Compute positive cone
                            probe_y = uvw_inflection + math::vec<3>(x, y, z);
                            cell_test_and_output(probe_y);

                            // Compute negative cone
                            probe_y = uvw_inflection + math::vec<3>(-x, y, z);
                            cell_test_and_output(probe_y);
                        }
                    }
                }
            }
        }

        // Resolve minimum candidate distance, if necessary
        if (kernRad < vol::max_cell_dist)
        {
            uint8_t min_dist = vol::max_cell_dist;
            for (uint16_t j = 0; j < candidate_count; j++)
            {
                min_dist = std::min(min_dist, candidate_distances[j]);
            }
            volumes[0].cell_occupancy[i] |= (vol::max_cell_dist << vol::occupancy_bits);
        }
        else
        {
            volumes[0].cell_occupancy[i] |= (vol::max_cell_dist << vol::occupancy_bits);
        }
    }
    mem::deallocate_tracing(vol::max_cell_dist * vol::max_cell_dist); // Release temporary per-kernel candidate distance array

    // Update transform metadata; eventually this should be loaded from disk
    volumes[0].metadata.pos = math::vec<3>(0.0f, 0.0f, 20.0f);
    volumes[0].metadata.orientation = math::vec<4>(0.0f, 0.0f, 0.0f, 1.0f);
    volumes[0].metadata.scale = math::vec<3>(4, 4, 4); // Boring regular unit scale

    // Update material metadata; eventually this should be loaded from disk
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
        const math::vec<3> extents = cur_vol.metadata.scale * 0.5f; // Extents from object origin
        const math::vec<3> pos = cur_vol.metadata.pos;
        const math::vec<4> orientation = cur_vol.metadata.orientation;

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
            *vol_nfo_out = &volumes[i].metadata;
            return true;
        }
    }
    return false;
}
