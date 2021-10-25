
import ui;
import camera;
import <chrono>;
#include <windows.h>
#include "resource.h"
#undef min;
#undef max;
import mem;
import tracing;
import parallel;
import geometry;
import vox_ints;

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
    geometry::init();

    // Create the application window
    if (!ui::window_setup((void*)hInstance, nCmdShow, (void*)WndProc, szWindowClass, szTitle)) return FALSE;

    // Launch drawing work
    parallel::launch(tracing::trace);

    // Load window/desktop renderer
    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_VOXSCULPT));

    // Main message loop:
    u64 frameCtr = 0;
    MSG msg;
    backBuf = mem::allocate_tracing<u32>(camera::digital_colors_footprint);
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        // Update colors for rendering
        // Just one tile at a time, so we spend less time blocking render code ^_^
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

        // Present new colors on window wake/message
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            InvalidateRect(reinterpret_cast<HWND>(platform::osGetWindowHandle()), NULL, false);
        }
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
        case WM_DESTROY:
            PostQuitMessage(0);
            break;
        default:
            return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}
