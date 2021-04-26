
#include "mem.h"
#include "camera.h"

mem::address mem::tracing_arena; // Active memory for rendering operations, owned by render threads
uint64_t mem::tracing_footprint; // Resolved at runtime
mem::address mem::pool; // All the memory controlled by rtSoft
uint64_t mem::max_footprint;

void mem::init()
{
   tracing_footprint = camera::max_footprint;
   max_footprint = tracing_footprint; // Enough memory for separate tracing/io arenas
   pool = _aligned_malloc(max_footprint, sizeof(DWORD));//HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, max_footprint); // Full up-front allocation
   tracing_arena = pool;
}

void mem::deinit()
{
   _aligned_free(pool);
}