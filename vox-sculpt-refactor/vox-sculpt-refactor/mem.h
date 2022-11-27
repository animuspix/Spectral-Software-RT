#pragma once

#include "platform.h"

class mem
{
public:
    static constexpr uint32_t kilobyte = 1024;
    static constexpr uint64_t megabyte = kilobyte * kilobyte;
    static constexpr uint64_t gigabyte = megabyte * megabyte;
    typedef void* address;
    static void init();

    static void* generic_tracing_allocator(uint32_t num_bytes);

    template<typename type_allocating>
    static type_allocating* allocate_tracing() // No alignment support atm, check with Athru as needed
    {
        return (type_allocating*)generic_tracing_allocator(sizeof(type_allocating));
    }

    template<typename type_allocating>
    static type_allocating* allocate_tracing(uint32_t custom_footprint) // For arrays, hidden padding, etc
    {
        return (type_allocating*)generic_tracing_allocator(custom_footprint);
    }

    static void deallocate_tracing(uint32_t num_bytes); // For short-term allocations needed by numerical arrays &c - most data should live until program exit
                                                        // Deallocates data from the end of our buffer, so every [deallocate] should have an earlier, matching [allocate] in the same scope

    static void deinit();
};