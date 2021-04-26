
#include "tracing.h"
#include "camera.h"
#include "parallel.h"
#include "math.h"
#include "sampler.h"
#include "io.h"

static uint32_t sample_ctr[parallel::numTiles] = {};
std::atomic_bool draws_running = true;
void tracing::trace(int16_t width, int16_t height, int16_t minX, int16_t minY, int16_t tileNdx)
{
   while (draws_running)
   {
      if (!parallel::drawFinished[tileNdx] && !io::present_switch)
      {
         // No actual rays for now, just a basic curve to validate our pipeline
         int16_t xMax = minX + parallel::tile_width;
         int16_t yMax = minY + parallel::tile_height;
         sample_ctr[tileNdx]++; // Starting another sample for every pixel in the current tile
         for (int32_t y = minY; y < yMax; y++)
         {
            for (int32_t x = minX; x < xMax; x++)
            {
               // Tracing output ends with "analog" writes
#define DEMO_SPECTRAL_INTEGRATION
//#define DEMO_FILM_RESPONSE
#ifdef DEMO_HYPERBOLA
               float distX = ((float)abs(abs(x - ui::image_centre_x) - ui::image_centre_x)) / (float)ui::image_centre_x;
               float distY = ((float)abs(abs(y - ui::image_centre_y) - ui::image_centre_y)) / (float)ui::image_centre_y;
               float dist = 1.0f - sqrt(distX * distX + distY * distY);
               camera::analog_colors[y * ui::window_width + x].rho = dist;
#elif defined(DEMO_AND)
               camera::analog_colors[y * ui::window_width + x].rho = (x & y) / (float)y;
#elif defined(DEMO_XOR)
               camera::analog_colors[y * ui::window_width + x].rho = ((x % 1024) ^ (y % 1024)) / 1024.0f;
#elif defined(DEMO_NOISE)
               uint32_t sample[4];
               sampler::rand_streams[tileNdx].next(sample); // Three wasted values :(
               camera::analog_colors[y * ui::window_width + x].rho = (sample[0] ^ sample[1] ^ sample[2] ^ sample[3]) / 4294967295.0f;
#elif defined (DEMO_FILM_RESPONSE) // Rainbow gradient test
               uint32_t pixel_ndx = y * ui::window_width + x;
               camera::sensor_response(camera::spectrum((float)x / (float)ui::window_width, 1.0f), pixel_ndx, 1.0f);
               camera::tonemap_out(pixel_ndx);
#elif defined (DEMO_SPECTRAL_INTEGRATION)
               float sample[4];
               sampler::rand_streams[tileNdx].next(sample);
               camera::spectrum s(sample[0], 1.0f); // Random spectral sample, going to use a QMC sequence here eventually~

               // Lens sampling + scene traversal will happen here
               ///////////////////////////////////////////////////

               // Compute sensor response + apply sample weight (composite of integration weight for spectral accumulation,
               // lens-sampled filter weight for AA, and path index weights from ray propagation)
               uint32_t pixel_ndx = y * ui::window_width + x;
               camera::sensor_response(s, pixel_ndx, sample_ctr[tileNdx]);

               // Map resolved sensor responses back into tonemapped RGB valeus we can store for output
               camera::tonemap_out(pixel_ndx);
#endif
            }
         }

         // Free-up resources for other threads
         //std::this_thread::sleep_for(std::chrono::milliseconds(1));
         parallel::drawFinished[tileNdx] = true;
      }
   }
}

void tracing::stop_tracing()
{
   draws_running = false;
}