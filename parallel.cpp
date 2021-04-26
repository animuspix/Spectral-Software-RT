
#include "parallel.h"
#include "tracing.h"
#include "io.h"
#include "sampler.h"

std::thread parallel::draw[parallel::numTiles];
std::atomic_bool parallel::drawFinished[parallel::numTiles]; // "Finished" is per-frame, not for the whole program
std::thread parallel::ioWorker;

void parallel::launch()
{
   // Initialize random-number-generators
   for (uint8_t i = 0; i < (uint8_t)parallel::numTiles; i++)
   {
      sampler::rand_streams[i].init(i);
   }

   // Input/output worker is constantly waiting for tile updates, copying them out
   io::start_polling(ioWorker);

   // Draw workers are constantly integrating the scene
   for (uint16_t i = 0; i < numTiles; i++)
   {
      draw[i] = std::thread(tracing::trace, (int16_t)tile_width, (int16_t)tile_height, (int16_t)tileMinX(i), (int16_t)tileMinY(i), i);
   }
}

void parallel::stop_work()
{
   tracing::stop_tracing();
   for (uint16_t i = 0; i < numTiles; i++)
   {
      draw[i].join();
   }
   io::stop_polling();
   ioWorker.join();
}
