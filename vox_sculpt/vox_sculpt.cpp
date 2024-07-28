
import ui;
import camera;
import <chrono>;
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "resource.h"
#undef min;
#undef max;
import mem;
import tracing;
import parallel;
import geometry;
import platform;
import vox_ints;
import updater;

#define MAX_LOADSTRING 100

// Global Variables:
								// current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name
decltype(camera::digital_colors) backBuf; // Back-buffer for window output (separated from live image output to allow asynchronous rendering/presentation)

// Forward declarations of functions included in this code module:
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	// Initialize global strings
	LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadStringW(hInstance, IDC_VOXSCULPT, szWindowClass, MAX_LOADSTRING);

	// Initialize rendering systems
	mem::init();
	camera::init();
	parallel::init();
	tracing::init(); // Leave a gap between parallel initialization and the first system that needs access to our thread tiles,
	// so we avoid trying to launch work before threads are ready
	geometry::init(camera::inverse_lens_sample);

	// Create the application window
	if (!ui::window_setup((void*)hInstance, nCmdShow, (void*)WndProc, szWindowClass, szTitle)) return FALSE;

	// Launch drawing work
	parallel::launch(tracing::trace);

	// Load window/desktop renderer
	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_VOXSCULPT));

	//#define TIMED_TRACING
#ifdef TIMED_TRACING
	bool tracing_prepass_completed = false;
	double rt_t = platform::osGetCurrentTimeSeconds();
#endif
	double t = platform::osGetCurrentTimeMilliSeconds();
	double dt = 0;
	u32 frontend_ticks = 0;
	u32 present_interval = 20;

	// Main message loop:
	MSG msg;
	backBuf = mem::allocate_tracing<u32>(camera::digital_colors_footprint);
	while (GetMessage(&msg, nullptr, 0, 0))
	{
		// Update colors for rendering
		// Just one tile at a time, so we spend less time blocking render code ^_^
		// Modified again - checking tiles every 30ms instead of every frame
		if (frontend_ticks % present_interval == 0) // Aiming to scan for copies every 30ms, ~30fps
		{
			for (u8 i = 0; i < parallel::numTiles; i++)
			{
				if (parallel::tiles[i].messaging->load())
				{
					const u16 bound_y_min = static_cast<u16>(tracing::tracing_tile_positions[i].y());
					const u16 bound_y_max = static_cast<u16>(tracing::tracing_tile_bounds[i].y());
					const u16 bound_x_min = static_cast<u16>(tracing::tracing_tile_positions[i].x());
					const u16 tile_width = static_cast<u16>(tracing::tracing_tile_sizes[i].x());
					for (u16 j = bound_y_min; j < bound_y_max; j++)
					{
						u32 offs = (j * ui::window_width) + bound_x_min;
						memcpy(backBuf + offs, camera::digital_colors + offs, tile_width * sizeof(u32));
					}
					parallel::tiles[i].messaging->store(0);
				}
			}

			// Process renderer changes
			// (anything that might modify the view)
			render_updates::process();

			if (dt < 30) present_interval++;
			else present_interval--;
			present_interval = vmath::umax(vmath::umin(present_interval, 0x00f00000), 2u);
		}

		// Break & output render performance when timed ray-tracing is active
#ifdef TIMED_TRACING
		if (tracing::completed_tiles->load() == parallel::numTiles)
		{
			platform::osDebugLogFmt("volume tracing completed within %f seconds \n", platform::osGetCurrentTimeSeconds() - rt_t);
			platform::osDebugBreak();
		}
		if ((tracing::tile_prepass_completion->load() == parallel::numTiles) && !tracing_prepass_completed)
		{
			double t_trace_timer = platform::osGetCurrentTimeSeconds();
			platform::osDebugLogFmt("prepass/sky tracing completed within %f seconds \n", t_trace_timer - rt_t);
			rt_t = t_trace_timer; // Update timestamp before profiling volume tracing
			tracing_prepass_completed = true;
			//platform::osDebugBreak();
		}
#endif

		// Process window messages
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
			InvalidateRect(reinterpret_cast<HWND>(platform::osGetWindowHandle()), NULL, false);
		}

		// Increment tick/time counters (used for render copy scheduling)
		frontend_ticks++;

		// + Update delta-time (also used for render copy scheduling)
		const double curr_t = platform::osGetCurrentTimeMilliSeconds();
		dt = curr_t - t;
		t = curr_t;
	}

	// Clean-up, exit the program :)
	tracing::stop_tracing();
	parallel::stop_work();
	mem::deinit();
	return (int)msg.wParam;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE: Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_COMMAND:
	{
		int wmId = LOWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
	}
	break;
	case WM_PAINT:
	{
		// Ready to present, so send output bits off to the window surface
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hWnd, &ps);
		BITMAPINFO* bmi = (BITMAPINFO*)platform::osGetWindowBmi();
		u32 test = SetDIBitsToDevice(hdc, 0, 0, ui::window_width, ui::window_height, 0, 0, 0, ui::window_height, backBuf, const_cast<BITMAPINFO*>(bmi), DIB_RGB_COLORS);
		EndPaint(hWnd, &ps);
	}
	break;

	// See https://docs.microsoft.com/en-us/windows/win32/learnwin32/keyboard-input for windows platform input implementation
	// + https://docs.microsoft.com/en-us/windows/win32/inputdev/virtual-key-codes for windows key-codes
	case WM_KEYUP:
		if (!(GetKeyState(VK_ESCAPE) & 0x8000))
		{
			platform::osKeyUp(platform::VOX_SCULPT_KEYS::KEY_ESCAPE);
		}
		if (!(GetKeyState(0x5a) & 0x8000)) // 0x5a = Letter Z
		{
			platform::osKeyUp(platform::VOX_SCULPT_KEYS::KEY_Z);
		}
		if (!(GetKeyState(VK_LCONTROL) & 0x8000))
		{
			platform::osKeyUp(platform::VOX_SCULPT_KEYS::KEY_LCTRL);
		}
		if (!(GetKeyState(VK_LEFT) & 0x8000))
		{
			platform::osKeyUp(platform::VOX_SCULPT_KEYS::KEY_LEFT_ARROW);
		}
		if (!(GetKeyState(VK_RIGHT) & 0x8000))
		{
			platform::osKeyUp(platform::VOX_SCULPT_KEYS::KEY_RIGHT_ARROW);
		}
		if (!(GetKeyState(VK_UP) & 0x8000))
		{
			platform::osKeyUp(platform::VOX_SCULPT_KEYS::KEY_UP_ARROW);
		}
		if (!(GetKeyState(VK_DOWN) & 0x8000))
		{
			platform::osKeyUp(platform::VOX_SCULPT_KEYS::KEY_DOWN_ARROW);
		}
		break;

	case WM_KEYDOWN:
		if (GetKeyState(VK_ESCAPE) & 0x8000)
		{
			platform::osKeyDown(platform::VOX_SCULPT_KEYS::KEY_ESCAPE);

			// Instant quit for now, eventually this will hook into UI and prompt a warning pop-up/save dialog
			PostQuitMessage(0);
		}
		if (GetKeyState(0x5a) & 0x8000) // 0x5a = Letter Z
		{
			platform::osKeyDown(platform::VOX_SCULPT_KEYS::KEY_Z);
		}
		if (GetKeyState(VK_LCONTROL) & 0x8000)
		{
			platform::osKeyDown(platform::VOX_SCULPT_KEYS::KEY_LCTRL);
		}
		if (GetKeyState(VK_LEFT) & 0x8000)
		{
			platform::osKeyDown(platform::VOX_SCULPT_KEYS::KEY_LEFT_ARROW);
		}
		if (GetKeyState(VK_RIGHT) & 0x8000)
		{
			platform::osKeyDown(platform::VOX_SCULPT_KEYS::KEY_RIGHT_ARROW);
		}
		if (GetKeyState(VK_UP) & 0x8000)
		{
			platform::osKeyDown(platform::VOX_SCULPT_KEYS::KEY_UP_ARROW);
		}
		if (GetKeyState(VK_DOWN) & 0x8000)
		{
			platform::osKeyDown(platform::VOX_SCULPT_KEYS::KEY_DOWN_ARROW);
		}
		break;

	case WM_DESTROY:
		PostQuitMessage(0);
		break;

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}
