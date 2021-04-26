#pragma once

#include "camera.h"
#include "parallel.h"

class io
{
   public:
   // Launch polling thread
   static void start_polling(std::thread& target_thread);
   static void stop_polling();

   // Semaphore to control publishing to the window surface
   static std::atomic_bool present_switch;
};