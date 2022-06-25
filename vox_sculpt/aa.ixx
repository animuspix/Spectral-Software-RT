export module aa;
import vmath;
import vox_ints;

export namespace aa
{
	constexpr u32 samples_x = 2;
	constexpr u32 samples_y = 2;
	constexpr u32 max_samples = samples_x * samples_y;
	float blackman_harris_weights[max_samples] = {};
	void init_filter_weights()
	{
		for (u32 i = 0; i < max_samples; i++)
		{
			// Blackman-Harris alpha parameters
			// Parameter values taken from:
			// https://en.wikipedia.org/wiki/Window_function
			// (section "A list of window functions", subsection "Blackman-Harris window")
			const vmath::vec<2> alph0 = vmath::vec<2>(0.35875f);
			const vmath::vec<2> alph1 = vmath::vec<2>(0.48829f);
			const vmath::vec<2> alph2 = vmath::vec<2>(0.14128f);
			const vmath::vec<2> alph3 = vmath::vec<2>(0.01168f);

			// Core Blackman-Harris filter function
			vmath::vec<2> sample_xy = vmath::vabs(vmath::vec<2>(i % samples_x, i / samples_x) - vmath::vec<2>(samples_x * 0.5f, samples_y * 0.5f)); // Transform coordinates to absolute pixel distances
			vmath::vec<2> ratio = vmath::vec<2>(vmath::pi * sample_xy) / float(max_samples - 1);
			vmath::vec<2> filtv = alph0 - (alph1 * vmath::cos(2.0f * ratio)) +
										  (alph2 * vmath::cos(4.0f * ratio)) -
										  (alph3 * vmath::cos(6.0f * ratio));
			blackman_harris_weights[i] = filtv.x() * filtv.y();
		}
	}
	float blackman_harris_weight(vmath::vec<2> sample_xy)
	{
		// Transform coordinates to absolute pixel distances
		//const vmath::vec<2> refInput = sample_xy;
		sample_xy = vmath::vabs(sample_xy + vmath::vec<2>(samples_x * 0.5f, samples_y * 0.5f));
		//
		//// Compute indices vs
		//vmath::vec<2> sample_indices = vmath::vec<2>(u32(sample_xy.x()), u32(sample_xy.y()));
		//vmath::vec<2> sample_noise = sample_xy - sample_indices;
		//const float centroid = blackman_harris_weights[u32(sample_indices.y() * samples_x) + u32(sample_indices.x())];
		////const float upper = blackman_harris_weights[u32((sample_indices.y() + 1) * samples_x) + u32(sample_indices.x())];
		////const float right = blackman_harris_weights[u32(sample_indices.y() * samples_x) + u32(sample_indices.x() + 1)];
		//return centroid;//vmath::lerp(vmath::lerp(centroid, upper, sample_noise.y()), right, sample_noise.x());

		// Transform coordinates to absolute pixel distances
		//sample_xy = vmath::vabs(sample_xy - vmath::vec<2>(samples_x, samples_y));

		// Blackman-Harris alpha parameters
		// Parameter values taken from:
		// https://en.wikipedia.org/wiki/Window_function
		// (section "A list of window functions", subsection "Blackman-Harris window")
		//const vmath::vec<2> alph0 = vmath::vec<2>(0.35875f);
		//const vmath::vec<2> alph1 = vmath::vec<2>(0.48829f);
		//const vmath::vec<2> alph2 = vmath::vec<2>(0.14128f);
		//const vmath::vec<2> alph3 = vmath::vec<2>(0.01168f);
		//
		//// Core Blackman-Harris filter function
		//vmath::vec<2> ratio = vmath::vec<2>(vmath::pi * sample_xy) / float(max_samples - 1);
		//vmath::vec<2> filtv = alph0 - (alph1 * vmath::cos(2.0f * ratio)) +
		//							  (alph2 * vmath::cos(4.0f * ratio)) -
		//							  (alph3 * vmath::cos(6.0f * ratio));
		//return (filtv.x() * filtv.y());

		//return vmath::fsqrt()

		constexpr float sigma = 0.3f;
		constexpr float half_max_samples = max_samples / 2;
		// float n = 1;
		// frac *= frac;
		// float monodim = vmath::fexp(-0.5f * frac);

		// n == sample_xy
		vmath::vec<2> frac = (sample_xy - half_max_samples) / (sigma * half_max_samples);
		frac *= frac;
		vmath::vec<2> filtv = vmath::vexp(-0.5f * frac);
		return (filtv.x() * filtv.y());
	}
	vmath::vec<2> supersample(float film_x, float film_y)
	{
		return vmath::vec<2>(film_x * samples_x, film_y * samples_y);
	}
	vmath::vec<2> jitter(float rand_u, float rand_v)
	{
		// Jitter inside the sampling grid
		const float x_jitter = (rand_u * aa::samples_x) - (aa::samples_x * 0.5f);
		const float y_jitter = (rand_v * aa::samples_y) - (aa::samples_y * 0.5f);
		return vmath::vec<2>(x_jitter, y_jitter);
	}
}