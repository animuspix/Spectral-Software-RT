#pragma once

#include "ui.h"
#include <stdint.h>
#include <windows.h>
#undef min
#undef max

class mem
{
   public:
   static constexpr uint32_t kilobyte = 1024;
   static constexpr uint64_t megabyte = kilobyte * kilobyte;
   static constexpr uint64_t gigabyte = megabyte * megabyte;
   typedef void* address;
   static uint8_t* tracing_arena; // Active memory for rendering operations, owned by render threads (no other arenas in use atm)
   static uint64_t tracing_footprint; // Resolved at runtime
   static address pool; // All the memory controlled by rtSoft
   static uint64_t max_footprint;
   static uint64_t alloc_offs; // Offset for dynamic allocations; static allocations are thoretically possible but difficult at scale without
                               // compiler support
   static void init();
   template<typename type_allocating>
   static type_allocating* allocate_tracing(uint32_t num_bytes) // No alignment support atm, check with Athru as needed
   {
      type_allocating* ret_ptr = (type_allocating*)(tracing_arena + alloc_offs);
      alloc_offs += num_bytes;
      return ret_ptr;
   }
   static void deinit();
};