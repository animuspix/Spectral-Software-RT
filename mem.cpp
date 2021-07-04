
#include "mem.h"
#include "camera.h"
#include "geometry.h"

uint8_t* mem::tracing_arena; // Active memory for rendering operations, owned by render threads
mem::address mem::pool; // All the memory controlled by rtSoft
uint64_t mem::max_footprint;
uint64_t mem::alloc_offs;

void mem::init()
{
   max_footprint = UINT32_MAX; // Arbitrary init size, since c++ doesn't seem to have great upfront allocation support and i kinda super ceebs continuing to
                               // manage system footprints myself
   pool = _aligned_malloc(max_footprint, sizeof(DWORD));//HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, max_footprint); // Full up-front allocation
   tracing_arena = (uint8_t*)pool;
   alloc_offs = 0;
}

void mem::deinit()
{
   _aligned_free(pool);
}