#include "mem.h"

uint8_t* tracing_arena; // Active memory for rendering operations, owned by render threads (no other arenas in use atm)
uint64_t tracing_footprint; // Resolved at runtime
mem::address pool; // All the memory controlled by rtSoft
uint64_t max_footprint;
uint64_t alloc_offs; // Offset for dynamic allocations; allocations are thoretically possible but difficult at scale without
// compiler support

void mem::init()
{
    static_assert(sizeof(void*) == 8, "vox_sculpt needs at least 4GB of memory - 32bit builds will fail to allocate and crash early in startup");
    max_footprint = 0x100000000; // Arbitrary init size, since c++ doesn't seem to have great upfront allocation support and i kinda super ceebs continuing to
    // manage system footprints myself
    pool = platform::osMalloc(max_footprint, sizeof(uint32_t)); //HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, max_footprint); // Full up-front allocation
    tracing_arena = (uint8_t*)pool;
    alloc_offs = 0;
}

void* mem::generic_tracing_allocator(uint32_t num_bytes)
{
    void* ret_ptr = (tracing_arena + alloc_offs);
    alloc_offs += num_bytes;
    return ret_ptr;
}

void mem::deallocate_tracing(uint32_t num_bytes)
{
    alloc_offs -= num_bytes;
}

void mem::deinit()
{
    platform::osFree(pool);
}

