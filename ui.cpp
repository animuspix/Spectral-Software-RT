
#include <assert.h>
#include "ui.h"
#include "camera.h"

BITMAPINFO ui::bmi;
HWND ui::hWnd;
BOOL ui::window_setup(HINSTANCE hInstance, int nCmdShow, WNDPROC wndProcSig, WCHAR* wndClassName, WCHAR* wndTitle)
{
   // Register window class
   WNDCLASSEXW wcex;
   wcex.cbSize = sizeof(WNDCLASSEX);
   wcex.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
   wcex.lpfnWndProc = wndProcSig;
   wcex.cbClsExtra = 0;
   wcex.cbWndExtra = 0;
   wcex.hInstance = hInstance;
   wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_RTSOFTRENDER));
   wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
   wcex.hbrBackground = (HBRUSH)(GetStockObject(LTGRAY_BRUSH));
   wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_RTSOFTRENDER);
   wcex.lpszClassName = wndClassName;
   wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
   RegisterClassExW(&wcex);

   // Create window
   hWnd = CreateWindowW(wndClassName, wndTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);
   if (!hWnd)
   {
      return FALSE;
   }
   else
   {
      ShowWindow(hWnd, nCmdShow);
      UpdateWindow(hWnd);
   }

   // Cache bitmap info
   BITMAPINFO nfo;
   ZeroMemory(&nfo, sizeof(BITMAPINFO));
   nfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
   nfo.bmiHeader.biWidth = window_width;
   nfo.bmiHeader.biHeight = window_height;
   nfo.bmiHeader.biPlanes = 1;
   nfo.bmiHeader.biBitCount = 32;
   nfo.bmiHeader.biCompression = BI_RGB;
   nfo.bmiHeader.biSizeImage = 0;
   nfo.bmiHeader.biXPelsPerMeter = 0; // 3840 / 36cm
   nfo.bmiHeader.biYPelsPerMeter = 0; // 2160 / 16cm
   nfo.bmiHeader.biClrUsed = FALSE;
   nfo.bmiHeader.biClrImportant = FALSE;
   bmi = nfo;
   return TRUE;
}
