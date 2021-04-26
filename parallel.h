#pragma once

#include <thread>
#include "ui.h"

class parallel
{
public:
   static constexpr uint16_t numTilesX = 3;
   static constexpr uint16_t numTilesY = 3;
   static constexpr uint16_t numTiles = numTilesX * numTilesY;
   static constexpr uint16_t tile_width = ui::window_width / numTilesX;
   static constexpr uint16_t tile_height = ui::window_height / numTilesY;
   static uint16_t tileMinX(uint16_t i) { return (i % numTilesX) * tile_width; };
   static uint16_t tileMinY(uint16_t i) { return (i / numTilesY) * tile_height; };
   static std::thread draw[numTiles];
   static std::atomic_bool drawFinished[numTiles]; // "Finished" is per-frame, not for the whole program
   static std::thread ioWorker;
   static void launch();
   static void stop_work();
};