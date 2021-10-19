export module ui;

import vox_ints;
import platform;

// should integrate imgui here eventually

export namespace ui
{
    constexpr u16 window_width = 1920;
    constexpr u16 window_height = 1080;
    constexpr u32 window_area = window_width * window_height;
    constexpr u16 image_centre_x = window_width / 2;
    constexpr u16 image_centre_y = window_height / 2;
    bool window_setup(void* hInstance, int nCmdShow, void* wndProcSig, wchar_t* wndClassName, wchar_t* wndTitle)
    {
        return platform::osWindowSetup(hInstance, nCmdShow, wndProcSig, wndClassName, wndTitle, window_width, window_height);
    }
};