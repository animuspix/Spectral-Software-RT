export module shader;
import gpu_texture;
import gpu_buffer;
import vox_ints;
import <concepts>;

// C++ code dynamically transpiled into HLSL through lambdas & operator magic on custom types

namespace shader
{
	export template<u32 numBufBindings, u32 numTexBindings, u32 numTexOutputs>
	struct bindlist
	{
		gpu_buffer::buf local_buffers[numBufBindings];
		gpu_texture::hardware_tex local_textures[numTexBindings];
		gpu_texture::staging_tex transfer[numTexOutputs]; // To enable scheduling copies without a swapchain
	};

	enum EXEC_MODE
	{
		SINGLE_THRD_EMU, // Single-threaded cpu execution, for debugging
		D3D11
	};
	export template<EXEC_MODE exec, u32 numBufBindings, u32 numTexBindings, u32 numTexOutputs, u32 groupThreadsX, u32 groupThreadsY>
	struct pass
	{
		// Custom types - all operators overload to either do normal work on cpu, or codegen for gpu
		struct gpu_f32
		{
			float value = 0.0f;
			operator+();
			operator-();
			operator*();
			operator/();
		};
		struct gpu_u32
		{
			u32 value = 0;
			operator+();
			operator-();
			operator*();
			operator/();
		};
		struct gpu_vec2
		{
			vmath::vec<2> value;
			operator+();
			operator-();
			operator*();
			operator/();
		};
		struct gpu_vec3
		{
			vmath::vec<2> value;
			operator+();
			operator-();
			operator*();
			operator/();
		};
		struct gpu_vec4
		{
			vmath::vec<2> value;
			operator+();
			operator-();
			operator*();
			operator/();
		};

		// Ctor takes a lambda and checks exec-mode; if gpu, run immediately and codegen into a buffer,
		// if cpu store and run directly on dispatch
		pass(decltype(src_xplatform) src, decltype(bindings) _bindings)
		{
			if constexpr (exec == D3D11)
			{
				// Codegen ^_^
				src(platform_src, bindings); // Operators on gpu_f32 and gpu_u32 allow cross-platform source to double as a code generator

				// Outsource compilation &c to platform code
				//platform_gpu::build_pass()
			}
			else if constexpr (exec == SINGLE_THRD_EMU)
			{
				src_xplatform = src;
			}
			bindings = _bindings;
		}
		void dispatch()
		{
			if constexpr (exec == D3D11)
			{
				//platform_gpu::dispatch(bindings.transfer[0].width / numGroupsX, bindings.transfer[0].height / numGroupsX, 1);
			}
			else if constexpr (exec == SINGLE_THRD_EMU)
			{
				src_xplatform = src;
			}
		}
		static constexpr u32 maxSrcLen = 2048; // No more than 2KB of generated logic supported per-shader
		char platform_src[maxSrcLen] = {};
		bindlist<numBufBindings, numTexBindings, numTexOutputs> bindings;
		private:
			std::function<void(char*, bindlist<numBufBindings, numTexBindings, numTexOutputs>)> src_xplatform;
	};
}