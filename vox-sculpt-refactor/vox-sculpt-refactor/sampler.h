#pragma once

// A generator for different ray-tracing and simulation distributions, including a CBRNG (Philox), a to-be-implemented QMC sequence (probably Sobol), and various useful hashes
// Sobol implementation is from Physically Based Rendering, 3rd Edition (Pharr, Jakob, Humphreys); Philox is described in the paper Parallel Random Numbers: As Easy as 1, 2, 3,
// available here,
// http://www.thesalmons.org/john/random123/papers/random123sc11.pdf
// and a provided API, available here:
// http://www.deshawresearch.com/downloads/download_random123.cgi/
// This implementation is one of the slightly worse ones - 4x32-7 (four chunks of 32-bit state, seven rounds iteration/step), trading off numerical reliabilty for easier implementation
// (no need for a custom software multiplier for 64-bit products vs 128-bit)

// Hash implementations are all random ones from the internet, sources given below
// :)
//////////////////////////////////////////////

#include <stdint.h>
#include <intrin.h>
namespace sampler
{
    // Modified integer hash from iq, found in Nimitz's WebGL2 Quality Hashes Collection
    // (https://www.shadertoy.com/view/Xt3cDn)
    // Original from:
    // https://www.shadertoy.com/view/4tXyWN
    uint32_t ihashIII(uint32_t i);

    // Lightly modified integer hash from Thomas Mueller, found here:
    // https://stackoverflow.com/questions/664014/what-integer-hash-function-are-good-that-accepts-an-integer-hash-key/12996028#12996028
    uint32_t tmhash(uint32_t x);

    // Four-wide vector hash, modified from Nimitz's WebGL2 Quality Hashes Collection
    // (https://www.shadertoy.com/view/Xt3cDn)
    // Named as a mix between the basis hash (XXHash32-derived, https://github.com/Cyan4973/xxHash)
    // and the output transformation (MINSTD, http://random.mat.sbg.ac.at/results/karl/server/node4.html)
    void xxminstd32(uint32_t i, uint32_t* out_x, uint32_t* out_y, uint32_t* out_z, uint32_t* out_w);

    // Wrapper for philox iteration + state management
    struct philox
    {
        uint32_t ctr[4];
        uint64_t key;
        void next(float* output);

        // Initialization assumes one seed/tile
        void init(uint8_t tileNdx);
    };
};