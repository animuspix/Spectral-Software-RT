
#include "path.h"

void path::push(path_vt vt)
{
    vts[front] = vt;
    front++;
    size++;
}

void path::resolve_path_weights(float* rho_out, float* pdf_out, float* response_out)
{
    float pdf = 1.0f;
    float response = 1.0f;
    *rho_out = vts[0].rho_sample;
    for (uint32_t i = 0; i < size; i++)
    {
        auto& bounce = vts[i];
        pdf *= bounce.pdf;
        response *= bounce.rho_weight;
    }
    *pdf_out = pdf;
    *response_out = response;

    // Force assets to render in solid colour for debugging
//#define HIGHLIGHT_SURFACES
#ifdef HIGHLIGHT_SURFACES
    if (vts[0].mat != nullptr)
    {
        *response_out = 1.0f;
        *rho_out = 0.85f;
    }
#endif
}

void path::clear()
{
    front = 0;
    size = 0;
}
