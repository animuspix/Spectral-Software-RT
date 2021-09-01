#pragma once

#include <stdint.h>
#include "mem.h"

template<typename elt_type, uint32_t max_num_elts>
struct tmp_vec
{
    elt_type* elts = nullptr;
    uint32_t curr_num_elts = 0;
    static constexpr uint32_t footprint = sizeof(elt_type) * max_num_elts;
    tmp_vec() // Takes [max_size * sizeof(elt_type)] bytes from preallocated memory with mem::allocate_tracing(...)
    {
        elts = mem::allocate_tracing<elt_type>(footprint);
    }
    ~tmp_vec() // Returns memory at end-of-stack (we use a linear allocator, so no medium-term allocations allowed); uses mem::deallocate_tracing(...)
    {
        mem::deallocate_tracing(footprint);
    }
    void push(elt_type elt) // Only linear, progressive insertions supported; random insertions are slow and insertions from the start of our temp buffer require
                             // data shuffles (also slow)
    {
        elts[curr_num_elts] = elt;
        curr_num_elts += 1;
    }
    elt_type& read(uint32_t ndx)
    {
        return elts[ndx];
    }
};
