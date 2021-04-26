#pragma once

#include "ui.h"
#include <stdint.h>
#include <windows.h>

class mem
{
   public:
   static constexpr uint32_t kilobyte = 1024;
   static constexpr uint64_t megabyte = kilobyte * kilobyte;
   static constexpr uint64_t gigabyte = megabyte * megabyte;
   typedef void* address;
   static address tracing_arena; // Active memory for rendering operations, owned by render threads (no other arenas in use atm)
   static uint64_t tracing_footprint; // Resolved at runtimele
   static address pool; // All the memory controlled by rtSoft
   static uint64_t max_footprint;
   static void init();
   static void deinit();
};