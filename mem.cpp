
#include "mem.h"
#include "camera.h"
#include "geometry.h"

uint8_t* mem::tracing_arena; // Active memory for rendering operations, owned by render threads
uint64_t mem::tracing_footprint; // Resolved at runtime
mem::address mem::pool; // All the memory controlled by rtSoft
uint64_t mem::max_footprint;
uint64_t mem::alloc_offs;

void mem::init()
{
   tracing_footprint = camera::max_footprint +
                       tracing::path_footprint +
                       geometry::geometry_footprint;
   max_footprint = tracing_footprint; // Everything in the tracing arena for now, potential for more arenas as needed (maybe if we add a d3d11 renderer for previs?)
   pool = _aligned_malloc(max_footprint, sizeof(DWORD));//HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, max_footprint); // Full up-front allocation
   tracing_arena = (uint8_t*)pool;
   alloc_offs = 0;
}

void mem::deinit()
{
   _aligned_free(pool);
}