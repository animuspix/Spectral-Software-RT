#pragma once

#include "math.h"
#include <vector>
#include "path.h"

class scene
{
   public:
   // Traverse the scene, pass path vertices back up to our pipeline so they can be integrated separately from scene traversal
   // (allowing for BDPT/VCM and other integration schemes besides regular unidirectional)
   static constexpr float SKY_BRIGHTNESS = 250.0f;
   static void isect(path::path_vt init_vt, path* vertex_output, uint32_t tileNdx);
};

