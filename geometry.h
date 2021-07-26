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

        // Volume metadata (material, transform information)
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

    // Volume octree, allowing for fast intersection tests
    struct accel
    {
        uint8_t* octree_chunks; // Adjacency list; first eight chunks in the first eight bits, then the next 64 chunks (9th-73rd bits), then the next 512 (74th - 585th bits)...etc.
        uint16_t* rank_lengths; // Lengths in bytes for each rank in the octree, for faster & easier lookups
        uint8_t num_ranks = 0;
        typedef uint64_t granularity_type; // 4x4x4 granularity, because having access to many voxels at once makes some things easier
                                           // (normals!), its the largest size we can still manipulate with hardware bitwise operators
                                           // (letting us traverse/transform the grid efficiently using different bitmasks), and its
                                           // still smaller than a cacheline :D
                                           // Practically only two valid scalar granularities (with exact cube roots); single-byte (2x2x2 bits) and eight-byte
                                           // (4x4x4 bits)
        static constexpr uint8_t granularity_bytes = sizeof(granularity_type); // Cell-state granularity, in bytes
        static constexpr uint8_t granularity_bits = granularity_bytes * 8; // Cell-state granularity, in bits
        uint8_t granularity_width = 0; // Minimal octree chunk width, solved at runtime (depends on the standard maths lib to find cubrt(granularity_bits))
        void init() // Not compile-time initializable, call this instead ^^
                    // This just performs basic data-structure setup; geometry/volume init populates the cell states + octree chunk bits
        {
            // - top has eight cells/eight bits (2x2x2), 512^3 voxels/cell
            // - next row has sixty-four cells (4x4x4), 256^3 voxels/cell
            // - third row has 512 cells (8x8x8), 128^3 voxels/cell
            // - fourth row has 4096 cells (16x16x16), 64^3 voxels/cell
            // - fifth row has 32768 cells (32x32x32), 32^3 voxels/cell
            // - sixth row has 262144 cells (64x64x64), 16^3 voxels/cell
            // - seventh row has 2097152 cells (128x128x128), 8^3 voxels/cell
            // - eighth row has 16777216 cells (256x256x256), 4^3 voxels/cell (64 bits, fits in a single uint64_t, no reason to go smaller than this)
            // - ninth row...2^3 (512x512x512)
            // - tenth row...1^3 (1024x1024x1024)

            // Each rank has half as many voxels/axis/chunk as the previous rank, but we want to stop
            // adding ranks after we pass our granularity limit; conveniently, the number of bits/byte of granularity matches the
            // change in volume per-rank (8x, since there's eight partitions (octants) in each block of 3D space), so the number
            // of bytes in our granularity type comes out to the same value as the number of ranks we need to take out to account
            // for it
            num_ranks = std::log2(vol::width) - granularity_bytes;
            rank_lengths = mem::allocate_tracing<uint16_t>(num_ranks);

            // Populate rank lengths, compute octree footprint
            uint32_t octree_footprint = 0;
            for (uint8_t i = 2; i < num_ranks; i++) // For an eight-rank octree the "zeroth" rank has eight elements (we skip the actual zeroth rank), and the
            {
                // Each rank has another eight subdivisions, thus (for the same volume as the actual voxel grid) the length of each rank in voxels is equal to 8^i
                const uint32_t len = std::pow(8, i - 1); // Length in voxels is conveniently equal to length in bits - take one exponent earlier so we can implicitly
                                                         // convert to bytes instead of dividing
                rank_lengths[i - 2] = len;
                octree_footprint += len;
            }

            // Separate out last two ranks for simplicity; n - 2 is another virtual layer where each bit is another octree chunk,
            // n - 1 is the voxel states themselves (sizeof(uint64_t) * std::pow(8, num_ranks - 1))
            uint32_t len_rank_second_last = std::pow(8, num_ranks - 3);
            uint32_t len_rank_last = std::pow(8, num_ranks - 1) * granularity_bytes;
            rank_lengths[num_ranks - 2] = len_rank_second_last;
            rank_lengths[num_ranks - 1] = len_rank_last;
            octree_footprint += len_rank_second_last;
            octree_footprint += len_rank_last;

            // Allocate octree data
            octree_chunks = mem::allocate_tracing<uint8_t>(octree_footprint);

            // Resolve minimum chunk/cell width
            granularity_width = std::floor(std::cbrt(granularity_bits)); // All granularities should have valid cube roots (i.e. resolve into regular cubes after sampling)
        }

        // State type for cell occupancies
        enum class GRID_CELL_STATE : uint8_t
        {
            OCCUPIED,
            UNOCCUPIED
        };

        // Test a given position + direction against the octree
        GRID_CELL_STATE query(math::vec<3> dir, math::vec<3> pos)
        {
            // Transform position to index, test; step to the edge of empty ranks, continue to resolve populated ranks
            // [update()] somewhat implements the uvw -> octree cell mapping :D
            // (no time to implement things rn)
        }

        // Highlight/mask the octree cells associated with the given index
        void update(uint32_t cell_ndx, GRID_CELL_STATE state)
        {
            // Always propagate signal for activated cells (activating parents is either good or harmless)
            if (state == GRID_CELL_STATE::OCCUPIED)
            {
                uint8_t rank_ctr = 0;
                uint32_t rank_offs = 0;
                uint32_t width_h = vol::width / 2;
                math::vec<3> uvw = vol::uvw_solver(cell_ndx);
                while (width_h > granularity_width)
                {
                    // Need to find time on the weekend to work out details here...
                    ///////////////////////////////////////////////////////////////

                    // Octants, for reference
                    // -----------
                    // 1    |    2
                    //      |
                    // -----|-----
                    //      |
                    // 3    |    4
                    // -----------
                    //
                    // -----------
                    // 5    |    6
                    //      |
                    // -----|-----
                    //      |
                    // 7    |    8
                    // -----------
                    /////////////////////

                    uvw -= math::vec<3>(width_h, width_h, width_h);
                    if (uvw.x() < 0 &&
                        uvw.y() < 0 &&
                        uvw.z() < 0) // Near corner
                    {
                        // Need to generalize this, no idea what to do for higher ranks
                        if (rank_ctr == 0) octree_chunks[0] |= 0b00000001;
                    }

                    // Need to add else-if/else branches for each octant here
                    // (or specify octant bounds in an array I can loop over later, probably simplest to do that)
                    /////////////////////////////////////////////////////////

                    // Halve width for every rank
                    width_h <<= 2;

                    // Increment rank offset
                    rank_offs += rank_lengths[rank_ctr];

                    // Increment rank counter
                    rank_ctr++;
                }
            }
            else // Only sometimes propagate quite cells (deactivating parents can have freaky consequences)
            {
                // No time to implement this either; same algorithm as above to find hierachy for the current cell,
                // but pass ranks into a queue as you go - if zeroing out [cell_ndx] also zeroes the bottom-most cell, work back through the
                // queue and zero all the appropriate bits
            }
        }
    };
    static accel* accelStructure;
    static vol* volume;
    static void init();
    static bool test(math::vec<3> dir, math::vec<3>* ro_inout, geometry::vol::vol_nfo** vol_nfo_out);
};

