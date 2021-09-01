
#include "tracing.h"
#include "camera.h"
#include "parallel.h"
#include "math.h"
#include "sampler.h"
#include "io.h"
#include "scene.h"

path* tracing::cameraPaths;
float* tracing::isosurf_distances;
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
                // Core path integrator, + demo effects
#define DEMO_SPECTRAL_PT
//#define DEMO_FILM_RESPONSE
//#define DEMO_XOR
//#define DEMO_NOISE
//#define DEMO_AND
//#define DEMO_HYPERBOLA
#ifdef DEMO_HYPERBOLA
               float distX = ((float)abs(abs(x - ui::image_centre_x) - ui::image_centre_x)) / (float)ui::image_centre_x;
               float distY = ((float)abs(abs(y - ui::image_centre_y) - ui::image_centre_y)) / (float)ui::image_centre_y;
               float dist = 1.0f - sqrt(distX * distX + distY * distY);
               uint32_t pixel_ndx = y * ui::window_width + x;
               camera::sensor_response(dist, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
               camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_AND)
               float rho = (x & y) / (float)y;
               uint32_t pixel_ndx = y * ui::window_width + x;
               camera::sensor_response(rho, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
               camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_XOR)
               float rho = ((x % 1024) ^ (y % 1024)) / 1024.0f;
               uint32_t pixel_ndx = y * ui::window_width + x;
               camera::sensor_response(rho, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
               camera::tonemap_out(pixel_ndx);
#elif defined(DEMO_NOISE)
               float sample[4];
               sampler::rand_streams[tileNdx].next(sample); // Three wasted values :(
               uint32_t pixel_ndx = y * ui::window_width + x;
               camera::sensor_response(sample[0], 1.0f, pixel_ndx, sample_ctr[tileNdx]);
               camera::tonemap_out(pixel_ndx);
#elif defined (DEMO_FILM_RESPONSE) // Rainbow gradient test
               uint32_t pixel_ndx = y * ui::window_width + x;
               camera::sensor_response((float)x / (float)ui::window_width, 1.0f, pixel_ndx, sample_ctr[tileNdx]);
               camera::tonemap_out(pixel_ndx);
#elif defined (DEMO_SPECTRAL_PT)
               float sample[4];
               sampler::rand_streams[tileNdx].next(sample); // Random spectral sample, should use QMC here

               // Intersect the scene
               uint32_t pixel_ndx = y * ui::window_width + x;
               scene::isect(camera::lens_sample((float)x, (float)y, sample[0]), cameraPaths + tileNdx, isosurf_distances + pixel_ndx, tileNdx);
               //scene::isect(camera::lens_sample(x, y, sample[0]), lightPaths[tileNdx]);

               // Integrate scene contributions (unidirectional for now)
               float rho, pdf, rho_weight;
               cameraPaths[tileNdx].resolve_path_weights(&rho, &pdf, &rho_weight); // Light/camera path merging decisions are performed while we integrate camera paths,
                                                                                   // so we only need to resolve weights for one batch

               // Apply path probabilitiy into sample weight
               rho_weight *= pdf;

               //if (cameraPaths[tileNdx].front > 0) DebugBreak();

               // Remove the current path from the backlog for this tile
               // (BDPT/VCM implementation will delay this until after separately tracing every path)
               cameraPaths[tileNdx].clear();
               //lightPaths[tileNdx].clear();

               // Compute sensor response + apply sample weight (composite of integration weight for spectral accumulation,
               // lens-sampled filter weight for AA, and path index weights from ray propagation)
               camera::sensor_response(rho, rho_weight, pixel_ndx, sample_ctr[tileNdx]);

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

void tracing::init()
{
   cameraPaths = mem::allocate_tracing<path>(sizeof(path) * parallel::numTiles);
   ZeroMemory(cameraPaths, sizeof(path) * parallel::numTiles);
   isosurf_distances = (float*)mem::allocate_tracing<float>(sizeof(float) * ui::window_area); // Eventually this will be per-subpixel instead of per-macropixel, but I'm not quite up to adding AA yet
   for (uint32_t i : countRange(0u, ui::window_area))
   {
       isosurf_distances[i] = -1.0f; // Reserve zero distance for voxels directly facing a grid boundary
   }
}
