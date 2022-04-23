export module gpu_buffer;
import vox_ints;

namespace gpu_buffer
{
	struct data
	{
		void* platform_handle;
	};

	struct shader_view
	{
		void* platform_handle;
	};

	struct buf
	{
		data _data;
		shader_view view;
		u32 length;
	};
};