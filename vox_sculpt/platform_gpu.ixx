export module platform_gpu;
import vox_ints;
#include <d3d11_4.h>

namespace platform_gpu
{
	// No fixed-function/raster support, driver abstraction just needs to support compute + buffers + textures
	// Should move this to d3d12 eventually...
	ID3D11Device4* device;
	ID3D11DeviceContext4* ctx;

	export void init()
	{
		// Create device
		// Create device context
	}

	// Arguments should be the basic resource summary needed to call the CreateX() D3D11 function
	// (stride, resource type, usage, etc)
	export void create_buffer(void* buffer_handle, void* buffer_view_handle) // All buffers are read-only
	{
		// Fill out buffer description, create buffer
		// Fill out buffer view description, create buffer
	}

	export void create_staging_texture(void* texture_handle) // No views for staging textures
	{
		// Fill out texture description, create texture
	}

	export void create_hardware_rw_texture(void* texture_handle, void* texture_view_handle) // All buffers are read/write
	{
		// Fill out texture description, create buffer
		// Fill out texture view description, create buffer
	}

	// Shader builds use DXC (need to look-up how to call that programmatically)
	export void build_shader(void** shader_handle)
	{
		char* shader_dxil = nullptr;
		u32 shader_len = 0;
		// DXIL generation with DXC...
		device->CreateComputeShader(shader_dxil, shader_len, nullptr, reinterpret_cast<ID3D11ComputeShader**>(shader_handle));
	}
	export void prepare_frame_state(u32 num_read_only_views, void** read_only_views, u32 num_rw_views, void** rw_views)
	{
		ctx->CSSetShaderResources(0, num_read_only_views, reinterpret_cast<ID3D11ShaderResourceView**>(read_only_views));
		ctx->CSSetUnorderedAccessViews(0, num_rw_views, reinterpret_cast<ID3D11UnorderedAccessView**>(rw_views), nullptr);
	}
	export void dispatch(void* shader_handle, u32 x, u32 y, u32 z)
	{
		ctx->CSSetShader(reinterpret_cast<ID3D11ComputeShader*>(shader_handle), nullptr, 0);
		ctx->Dispatch(x, y, z);
	}
}