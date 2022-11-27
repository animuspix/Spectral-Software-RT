#pragma once

#include "../../animuspix-libs/SimpleTiling/SimpleTiling/SimpleTiling.h"

#include "mem.h"
#include "sampler.h"
#include "platform.h"
#include "vmath.h"
#include <stdint.h>
#include <stdio.h>
#include <thread>

namespace parallel
{
    uint16_t numTilesX = 0;
    uint16_t numTilesY = 0;
    uint16_t numTiles = 0;

    sampler::philox* rand_streams = nullptr; // One PRNG stream/tile
    char** logs = nullptr; // Debug messages, printable after tasks have finished and threads are idle
    uint32_t* log_lengths = nullptr; // Amount of valid text per-log; reset after printing

    void init()
    {
        // Find maximum tile count for the current platform
        numTiles = simple_tiling::GetNumTilesTotal();
        numTilesX = simple_tiling::GetNumTilesX();
        numTilesY = simple_tiling::GetNumTilesY();

        // Allocate PRNG streams
        rand_streams = mem::allocate_tracing<sampler::philox>(sizeof(sampler::philox) * numTiles);

        // Allocate log destinations
        // Each tile gets up to 4MB of debug output
        // (can probably optimize to subdivide one huge char* instead of lots of cache-unfriendly pointers)
        logs = mem::allocate_tracing<char*>(numTiles);
        for (uint16_t i = 0; i < numTiles; i++)
        {
            logs[i] = mem::allocate_tracing<char>(4 * 1024 * 1024);
            platform::osClearMem(logs[i], 4 * 1024 * 1024);
        }

        // Allocate log-length tags
        // Each tag gives a length of valid/printable text for each log
        log_lengths = mem::allocate_tracing<uint32_t>(numTiles * sizeof(uint32_t));

        // Initialize random-number-generators
        for (uint8_t i = 0; i < (uint8_t)numTiles; i++)
        {
            rand_streams[i].init(i);
        }
    }

    char* get_log_dst(uint8_t tile_ndx)
    {
        return logs[tile_ndx] + log_lengths[tile_ndx];
    }

    template<typename...input_types>
    void append_tile_log(uint8_t tile_ndx, const char* str, input_types...inputs)
    {
        char* dst = get_log_dst(tile_ndx);
        uint16_t dLen = 0;
        while (str[dLen] != '\0')
        {
            dLen++;
        }
        sprintf_s(dst, dLen, str, inputs...);
        log_lengths[tile_ndx] += dLen;
    }
};
