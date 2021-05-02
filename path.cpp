
#include "path.h"

void path::push(path_vt vt)
{
   paths[front] = vt;
   front++;
   size++;
}

void path::resolve_path_weights(float* rho_out, float* pdf_out, float* response_out)
{
   float pdf = 1.0f;
   float response = 1.0f;
   *rho_out = paths[0].rho_sample;
   for (auto& bounce : paths)
   {
      pdf *= bounce.pdf;
      response *= bounce.rho_weight;
   }
   *pdf_out = pdf;
   *response_out = response;
}

void path::clear()
{
   front = 0;
   size = 0;
}
