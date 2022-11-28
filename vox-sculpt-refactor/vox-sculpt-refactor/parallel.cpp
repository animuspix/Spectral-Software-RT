
#include "parallel.h"
#include <cstdio>
#include <cstdarg>

uint16_t numTilesX = 0;
uint16_t numTilesY = 0;
uint16_t numTiles = 0;

sampler::philox* rand_streams = nullptr; // One PRNG stream/tile
char** logs = nullptr; // Debug messages, printable after tasks have finished and threads are idle
uint32_t* log_lengths = nullptr; // Amount of valid text per-log; reset after printing

uint32_t parallel::GetNumTilesTotal()
{
    return numTiles;
}

uint32_t parallel::GetNumTilesX()
{
    return numTilesX;
}

uint32_t parallel::GetNumTilesY()
{
    return numTilesY;
}

sampler::philox parallel::GetRNGStream(uint32_t tile_ndx)
{
    return rand_streams[tile_ndx];
}

void parallel::init()
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

char* parallel::get_log_dst(uint8_t tile_ndx)
{
    return logs[tile_ndx] + log_lengths[tile_ndx];
}

void parallel::append_log_fmted(char* dst, uint32_t len, uint32_t tile_ndx, const char* txt, ...)
{
    char txt_staging[1024] = {};
    va_list args;
    va_start(args, txt_staging);
    vprintf(txt_staging, args);
    va_end(args);

    sprintf_s(dst, len, txt_staging);
    log_lengths[tile_ndx] += len;
}
