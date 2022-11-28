#include "path.h"

void tracing::path::push(path_vt vt)
{
    if (front >= capacity) platform::osDebugBreak();
    vts[front] = vt;
    front++;
    size++;
}

void tracing::path::resolve_path_weights(float* rho_out, float* pdf_out, float* response_out, float* power_out)
{
    float pdf = 1.0f;
    float response = 1.0f;
    float power = 1.0f;
    *rho_out = vts[0].rho_sample;
    for (int16_t i = 0; i < size; i++)
    {
        auto& bounce = vts[i];
        pdf *= bounce.pdf;
        response *= bounce.rho_weight;
        power *= bounce.power;

        // Process angular energy loss between vertices (the last cos(theta) in the rendering equation)
        if (i < (size - 1)) power *= vts[i].dir.dot(vts[i + 1].dir);
    }
    *pdf_out = pdf;
    *response_out = response;
    *power_out = power;

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
