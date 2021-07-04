#pragma once

#include <stdint.h>
#include <functional>
#include "math.h"
#include "materials.h"

class path
{
public:
    struct path_vt
    {
        path_vt(math::vec<3> _dir, math::vec<3> _ori, float _pdf, float _rho, float _rho_weight) :
            dir(_dir), ori(_ori), pdf(_pdf), rho_weight(_rho_weight), rho_sample(_rho) {}
        math::vec<3> dir;
        math::vec<3> ori;
        float pdf = 1.0f; // Perspective-projected lens probabilities initialize at 100%; there's a proof somewhere, but naively
                          // we know that every sensor/film cell is sampled exactly once and every sensor/film cell *must* be sampled
                          // for a full image, so the relative probability for each sensor can never be less/greater than 1
        float rho_weight = 1.0f; // Totally dependant on the average scene SPD to be meaningful; initialize to 1 to hint that all colours
                                 // should be reflected by all rays by default
        float rho_sample = 0.5f; // Place path spectra near the white-point of our film response curve (see camera.h) by default
        materials::instance* mat = nullptr; // Material at the intersection point, to allow recalculating shading as needed for light/camera path connections
    };
    uint16_t front = 0;
    uint16_t size = 0;
    void push(path_vt vt); // Pass a vertex into the current path
    void resolve_path_weights(float* rho_out, float* pdf_out, float* response_out);
    void clear();
    static constexpr uint16_t capacity = 32; // At most 32 bounces/ray
    path_vt vts[capacity];
};
