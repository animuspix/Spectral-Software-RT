#pragma once

#include "vmath.h"
#include "path.h"
#include "geometry.h"
#include "parallel.h"
#include "platform.h"
#include "materials.h"
#include "spectra.h"
#include "lights.h"

class traversal
{
public:
    // Traverse the scene, pass path vertices back up to our pipeline so they can be integrated separately from scene traversal
    // (allowing for BDPT/VCM and other integration schemes besides regular unidirectional)
    static void iterate(tracing::path_vt init_vt, tracing::path* vertex_output, float* isosurf_dist, uint32_t tileNdx);
};
