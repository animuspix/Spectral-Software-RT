export module parallel;

import ui;
import platform;
import sampler;
import mem;
import vox_ints;
import <stdio.h>;

export namespace parallel
{
    constexpr u16 numTilesX = 3;
    constexpr u16 numTilesY = 3;
    constexpr u16 numTiles = numTilesX * numTilesY;
    platform::threads::osThread<platform::threads::thread_meta::TILE> tiles[numTiles];
    sampler::philox rand_streams[numTiles]; // One PRNG stream/tile
    char* logs[numTiles]; // Debug messages, printable after tasks have finished and threads are idle
    u32 log_lengths[numTiles] = {}; // Amount of valid text per-log; reset after printing
    void init()
    {
        // Initialize random-number-generators, thread-state fences, and worker threads
        // Each tile gets up to 4MB of debug output
        for (u8 i = 0; i < (u8)parallel::numTiles; i++)
        {
            rand_streams[i].init(i);
            logs[i] = mem::allocate_tracing<char>(4 * 1024 * 1024);
            platform::osClearMem(logs[i], 4 * 1024 * 1024);
        }
        platform::threads::osInitBatchProcessors(tiles, numTiles);
    }

    // Work centre can dispatch anything that takes x/y tile counts + a tile index
    void launch(void(*work)(u16 num_tiles_x, u16 num_tiles_y, u16 tile_ndx))
    {
        // Draw workers are constantly integrating the scene
        platform::threads::osAssignAndWakeBatchProcessors(work, tiles, numTilesX, numTilesY, numTiles);
    }
    char* get_log_dst(u8 tile_ndx)
    {
        return logs[tile_ndx] + log_lengths[tile_ndx];
    }
    template<typename...input_types>
    void append_tile_log(u8 tile_ndx, const char* str, input_types...inputs)
    {
        char* dst = get_log_dst(tile_ndx);
        u16 dLen = 0;
        while (str[dLen] != '\0')
        {
            dLen++;
        }
        sprintf_s(dst, dLen, str, inputs...);
        log_lengths[tile_ndx] += dLen;
    }
    void stop_work()
    {
        platform::threads::osTerminateBatchProcessors(tiles, numTiles);
    }
};