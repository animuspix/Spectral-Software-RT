#pragma once

#include "../../animuspix-libs/SimpleTiling/SimpleTiling/SimpleTiling.h"

#include "mem.h"
#include "sampler.h"
#include "platform.h"
#include "vmath.h"
#include <stdint.h>
#include <stdio.h>
#include <thread>

class parallel
{
public:
    static uint32_t GetNumTilesTotal();
    static uint32_t GetNumTilesX();
    static uint32_t GetNumTilesY();

    static sampler::philox& GetRNGStream(uint32_t tile_ndx);

    static void init();
    static char* get_log_dst(uint8_t tile_ndx);

    static void append_log_fmted(char* dst, uint32_t len, uint32_t tile_ndx, const char* txt, ...);

    template<typename...input_types>
    static void append_tile_log(uint8_t tile_ndx, const char* str, input_types...inputs)
    {
        char* dst = get_log_dst(tile_ndx);
        uint16_t dLen = 0;
        while (str[dLen] != '\0')
        {
            dLen++;
        }
        append_log_fmted(dst, dLen, tile_ndx, str, inputs);
    }
};
