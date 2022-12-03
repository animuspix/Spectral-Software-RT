#pragma once

#include "vmath.h"

namespace aa
{
	constexpr uint32_t samples_x = 16;
	constexpr uint32_t samples_y = 16;
	constexpr uint32_t max_samples = samples_x * samples_y;
	float blackman_harris_weight(vmath::vec<2> sample_xy);
	vmath::vec<2> supersample(float film_x, float film_y);
	vmath::vec<2> jitter(float rand_u, float rand_v);
}