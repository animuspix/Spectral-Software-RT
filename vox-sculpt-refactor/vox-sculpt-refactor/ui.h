#pragma once

#include <stdint.h>

// should integrate imgui here eventually

namespace ui
{
    constexpr uint16_t window_width = 1920;
    constexpr uint16_t window_height = 1080;
    constexpr uint32_t window_area = window_width * window_height;
    constexpr int16_t image_centre_x = window_width / 2;
    constexpr int16_t image_centre_y = window_height / 2;
};