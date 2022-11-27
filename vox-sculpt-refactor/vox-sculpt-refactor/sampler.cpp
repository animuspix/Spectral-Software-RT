
#include "sampler.h"

uint32_t sampler::ihashIII(uint32_t i)
{
    i = 1103515245U * ((i >> 1U) ^ i);
    i = 1103515245U * (i ^ (i >> 3U));
    return i ^ (i >> 16);
}

uint32_t sampler::tmhash(uint32_t x)
{
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    return (x >> 16) ^ x;
}

void sampler::xxminstd32(uint32_t i, uint32_t* out_x, uint32_t* out_y, uint32_t* out_z, uint32_t* out_w)
{
    const uint32_t PRIME32_2 = 2246822519U, PRIME32_3 = 3266489917U;
    const uint32_t PRIME32_4 = 668265263U, PRIME32_5 = 374761393U;
    uint32_t h32 = i + PRIME32_5;
    h32 = PRIME32_4 * ((h32 << 17) | (h32 >> (32 - 17))); //Initial testing suggests this line could be omitted for extra perf
    h32 = PRIME32_2 * (h32 ^ (h32 >> 15));
    h32 = PRIME32_3 * (h32 ^ (h32 >> 13));
    h32 ^= (h32 >> 16);

    // Outputs!
    // see http://random.mat.sbg.ac.at/results/karl/server/node4.html for details
    *out_x = h32;
    *out_y = h32 * 16807U;
    *out_z = h32 * 48271U;
    *out_w = h32 * 69621U;
}

void sampler::philox::next(float* output)
{
    for (uint32_t i = 0; i < 8; i++)
    {
        // Update state
        constexpr uint64_t m0 = 0xCD9E8D57; // Multiplier from the Random123 paper, page 7
        constexpr uint64_t m1 = 0xD2511F53; // Multiplier from the Random123 paper, page 7
        uint64_t mul0 = ctr[0] * m0;
        uint64_t mul1 = ctr[2] * m1;
        uint64_t mul1_hi = (mul1 << 32);
        ctr[0] = (mul1 >> 32) ^ ctr[1] ^ ((key << 32) >> 32);
        ctr[1] = (mul1 << 32) >> 32;
        ctr[2] = (mul0 >> 32) ^ ctr[2] ^ (key >> 32);
        ctr[3] = (mul0 << 32) >> 32;

        // Update key
        constexpr uint64_t bump = (0x9E3779B9 | ((uint64_t)0xBB67AE85 << 32));
        key += bump;
    }

    // Pass updated state back to the callsite
    output[0] = ctr[0] / 4294967295.0f;
    output[1] = ctr[1] / 4294967295.0f;
    output[2] = ctr[2] / 4294967295.0f;
    output[3] = ctr[3] / 4294967295.0f;
}

void sampler::philox::init(uint8_t tileNdx)
{
    xxminstd32(tileNdx, ctr, ctr + 1, ctr + 2, ctr + 3); // Seed state by hashing indices with xxminstd32 (see above)
    key = (((uint64_t)ihashIII(tileNdx) << 32) |
        ((uint64_t)tmhash(tileNdx))); // Partially seed keys with a different hash (not Wang or xxminstd32)
}