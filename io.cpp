
#include "io.h"
#include "parallel.h"
std::atomic_bool io::present_switch = false;

// Enable surface present after every tile has resolved
std::atomic_bool io_running = true;
void poll()
{
   while (io_running)
   {
      // Check if all tiles are ready for upload
      bool traced_screen = true; // Start-off true and deliberately check for unfinished tiles because we don't
                                 // want to start filtering/output unless every tile is available
                                 // (asynchronous uploads are possible, but can be awkward at lower framerates as
                                 // different parts of the image go out-of-sync with the others)
      for (uint32_t i = 0; i < parallel::numTiles; i++)
      {
         traced_screen = traced_screen && parallel::drawFinished[i].load();
      }

      // Enable rendering if so
      if (traced_screen)
      {
         io::present_switch.store(true);
      }
   }
}

void io::start_polling(std::thread& dest)
{
   dest = std::thread(poll);
}

void io::stop_polling()
{
   io_running = false;
}
