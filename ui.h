#pragma once

#include <stdint.h>
#include <windows.h>
#include "resource.h"

// should integrate imgui here eventually

class ui
{
public:
   static constexpr uint16_t window_width = 1920;
   static constexpr uint16_t window_height = 1080;
   static constexpr uint32_t window_area = window_width * window_height;
   static constexpr uint16_t image_centre_x = window_width / 2;
   static constexpr uint16_t image_centre_y = window_height / 2;
   static HWND hWnd;
   static BITMAPINFO bmi;
   static BOOL window_setup(HINSTANCE hInstance, int nCmdShow, WNDPROC wndProcSig, WCHAR* wndClassName, WCHAR* wndTitle);
};