
// Quarantine platform-specific headers in here (these are either not ported to modules yet, or contain code that hasn't been ported to modules,
// so they aren't trustworthy within modularized projects)
#include <thread>
#include <sdkddkver.h>
#include <Windows.h>
#undef min
#undef max
#include "resource.h"
#include <assert.h>
#include <atomic>
#include <chrono>
import vox_ints;
import mem;
import platform;

void platform::threads::osAtomicInt::init()
{
    //InitializeSRWLock((PSRWLOCK)&lock_data);
    messenger = &core_data;
    *messenger = 0;
}

void platform::threads::osAtomicInt::inc()
{
    InterlockedIncrement(messenger);
}

void platform::threads::osAtomicInt::dec()
{
    InterlockedDecrement(messenger);
}

long platform::threads::osAtomicInt::load()
{
    // Order-dependant atomic operations still need locks >.> (maybe! I'm hoping I'm wrong about that)
    // But we can make them faster using specialized read/writer accesses
    // https://docs.microsoft.com/en-us/windows/win32/sync/slim-reader-writer--srw--locks
    long value = 0;
    //AcquireSRWLockExclusive((PSRWLOCK)&lock_data);
    value = *messenger;
    //ReleaseSRWLockExclusive((PSRWLOCK)&lock_data);
    return value;
}

void platform::threads::osAtomicInt::store(long value)
{
    InterlockedExchange(messenger, value);
}

// Win32 expects a global function taking a single parameter for CreateThread(), but we want to make everything in our thread object accessible;
// we work around that by optionally creating an object which exposes the relevant members, but only for functions we nominate to see them (like the global thread_main)
// Type constraint for thread control inputs
template<typename callable_t>
concept thread_task_type = std::is_same_v<callable_t, platform::threads::thread_meta::worker_thrd_fn> ||
                           std::is_same_v<callable_t, platform::threads::thread_meta::tile_thrd_fn>;
template<typename paramset_t>
concept thread_paramset_type = std::is_same_v<paramset_t, platform::threads::thread_meta::worker_params> ||
                               std::is_same_v<paramset_t, platform::threads::thread_meta::tile_params>;
template<thread_task_type task_type>
struct osThreadControls
{
    PCONDITION_VARIABLE _cvar;
    PCRITICAL_SECTION _crit_section;
    task_type* _next_task;
    void* _work_params;
    platform::threads::osAtomicInt* _state;
    platform::threads::thread_meta::THREAD_TYPES _type;
    platform::threads::osAtomicInt* _thread_openness;
};

// Buffer of controls for batched threads
#define DEBUG_THRD_INDICES
#ifdef DEBUG_THRD_INDICES
static u32 indices[9] = {};
#endif
volatile u32* batched_thrd_indices;
osThreadControls<platform::threads::thread_meta::tile_thrd_fn>* batched_thrd_controls = nullptr;
void placeholder_work_tiles(u16, u16, u16) {}
#define DBG_THREAD_MAIN
#ifdef DBG_THREAD_MAIN
#pragma optimize ("", off)
#endif
DWORD thread_main_tiles()
{
    // Safely update indices across threads
    u32 thrd_ndx = *batched_thrd_indices;
    InterlockedIncrement(batched_thrd_indices);
#ifdef DEBUG_THRD_INDICES
    indices[thrd_ndx] = thrd_ndx;
#endif

    // Resolve per-thread data
    osThreadControls<platform::threads::thread_meta::tile_thrd_fn> tile_controls = batched_thrd_controls[thrd_ndx];
    PCRITICAL_SECTION crit = tile_controls._crit_section;
    PCONDITION_VARIABLE cvar = tile_controls._cvar;
    platform::threads::thread_meta::tile_thrd_fn* task = tile_controls._next_task;
    void* task_params = tile_controls._work_params;
    platform::threads::osAtomicInt* state = tile_controls._state;
    platform::threads::osAtomicInt* thread_openness = tile_controls._thread_openness;

    // Enter the thread's main loop
    while (thread_openness->load())
    {
        EnterCriticalSection(crit); // Critical sections seem simpler & better than mutexes, going by this SO answer:
                                     // https://stackoverflow.com/questions/2808617/difference-between-locks-mutex-and-critical-sections#2808734
        if (SleepConditionVariableCS(cvar, crit, INFINITE))
        {
            platform::threads::thread_meta::tile_params* paramset = (platform::threads::thread_meta::tile_params*)task_params;
            state->store(platform::threads::WORKING);
            ((platform::threads::thread_meta::tile_thrd_fn)*task)(paramset->workgroup_width, paramset->workgroup_height, paramset->workgroup_index);
            state->store(platform::threads::SLEEPING);
        }
        LeaveCriticalSection(crit);
    }
    return 0;
}
#ifdef DBG_THREAD_MAIN
#pragma optimize ("", on)
#endif

void placeholder_work_workers() {}
DWORD thread_main_worker(void* ctrl)
{
    //osThreadControls<platform::threads::thread_meta::worker_thrd_fn>* worker_controls = (osThreadControls<platform::threads::thread_meta::worker_thrd_fn>*)ctrl;
    //PCRITICAL_SECTION crit = worker_controls->_crit_section;
    //PCONDITION_VARIABLE cvar = worker_controls->_cvar;
    //platform::threads::thread_meta::worker_thrd_fn* task = worker_controls->_next_task;
    //void* task_params = worker_controls->_work_params;
    //platform::threads::osAtomicInt* task_finish_state = worker_controls->status;
    //platform::threads::osAtomicInt* thread_openness = worker_controls->_thread_openness;
    //platform::osDebugLogFmt("thread_openness dest address: %x\n", reinterpret_cast<u64>(worker_controls->_thread_openness));
    //while (thread_openness->load())
    //{
    //    EnterCriticalSection(crit); // Critical sections seem simpler & better than mutexes, going by this SO answer:
    //                                 // https://stackoverflow.com/questions/2808617/difference-between-locks-mutex-and-critical-sections#2808734
    //    if (SleepConditionVariableCS(cvar, crit, INFINITE))
    //    {
    //        //platform::threads::thread_meta::worker_params* paramset = (platform::threads::thread_meta::worker_params)controls._work_params;
    //        ((platform::threads::thread_meta::worker_thrd_fn)*task)(); // No parameters for worker threads
    //        //OutputDebugStringA("awakened");
    //    }
    //    LeaveCriticalSection(crit);
    //}
    return 0;
}

PCONDITION_VARIABLE batch_cvar = nullptr;
void platform::threads::osInitBatchProcessors(platform::threads::osThread<platform::threads::thread_meta::TILE>* workgroup, u16 workgroup_area)
{
    // Allocate, initialize batch scheduling controller
    if (batch_cvar == nullptr) // Batch-processor condition should only be initialized once, and must be deinitialized before being recreated
    {
        batch_cvar = mem::allocate_tracing<CONDITION_VARIABLE>(sizeof(CONDITION_VARIABLE));
        InitializeConditionVariable(batch_cvar); // One condition variable to run/sleep the whole workgroup
    }

    // Memory for thread controls is allocated exactly once
    if (batched_thrd_controls == nullptr)
    {
        batched_thrd_controls = mem::allocate_tracing<osThreadControls<platform::threads::thread_meta::tile_thrd_fn>>(sizeof(osThreadControls<platform::threads::thread_meta::tile_thrd_fn>) * workgroup_area);
        batched_thrd_indices = mem::allocate_tracing<u32>(sizeof(u32));
        *batched_thrd_indices = 0;
    }

    // Initialize per-thread items
    for (u16 i = 0; i < workgroup_area; i++)
    {
        if (!workgroup[i].osCheckInitialized())
        {
            // Cvars are shared between tile threads, individualized for worker threads
            workgroup[i].cvar = batch_cvar;

            // Allocate, initialize critical section
            workgroup[i].thread_guard = mem::allocate_tracing<CRITICAL_SECTION>(sizeof(CRITICAL_SECTION));
            InitializeCriticalSection((LPCRITICAL_SECTION)workgroup[i].thread_guard);

            // Allocate & initialize task status atomic
            workgroup[i].messaging = mem::allocate_tracing<platform::threads::osAtomicInt>(sizeof(platform::threads::osAtomicInt));
            workgroup[i].messaging->init();

            // Allocate & initialize thread state atomic
            workgroup[i].state = mem::allocate_tracing<platform::threads::osAtomicInt>(sizeof(platform::threads::osAtomicInt));
            workgroup[i].state->init();

            // Allocate & initialize thread openness atomic
            workgroup[i].thread_openness = mem::allocate_tracing<platform::threads::osAtomicInt>(sizeof(platform::threads::osAtomicInt));
            workgroup[i].thread_openness->init();
            workgroup[i].thread_openness->inc(); // Threads loop by default

            // Setup thread controls
            osThreadControls<platform::threads::thread_meta::tile_thrd_fn>& tile_controls = batched_thrd_controls[i];
            tile_controls._cvar = batch_cvar;
            tile_controls._crit_section = (PCRITICAL_SECTION)workgroup[i].thread_guard;
            tile_controls._next_task = &workgroup[i].next_task;
            tile_controls._work_params = &workgroup[i].next_task_params;
            tile_controls._state = workgroup[i].state;
            tile_controls._type = platform::threads::thread_meta::TILE;
            tile_controls._thread_openness = workgroup[i].thread_openness;
        }
        else
        {
            // Multiple initializations are not supported, and probably indicate a bug
            DebugBreak();
        }
    }

    // Kickstart threads
    for (u16 i = 0; i < workgroup_area; i++)
    {
        workgroup[i].handle = CreateThread(NULL, 0xffff, (LPTHREAD_START_ROUTINE)thread_main_tiles, NULL, NULL, NULL);
    }
};

void platform::threads::osInitWorker(platform::threads::osThread<platform::threads::thread_meta::WORKER>* worker)
{
    // Revisiting when I start needing individual worker threads
    ////////////////////////////////////////////////////////////

    // Initialize shared tile/worker members
    //worker->osSharedThreadInit();

    //osThreadControls<platform::threads::thread_meta::worker_thrd_fn> controls;
    //InitializeCriticalSection((LPCRITICAL_SECTION)worker->thread_guard);
    //InitializeConditionVariable((PCONDITION_VARIABLE)worker->cvar);
    //controls._cvar = (PCONDITION_VARIABLE)worker->cvar;
    //controls._crit_section = (PCRITICAL_SECTION)worker->thread_guard;
    //controls._next_task = (platform::threads::thread_meta::worker_thrd_fn*)placeholder_work_workers;
    //controls._task_finish_state = worker->task_finish_state;
    //controls._type = platform::threads::thread_meta::TILE;
    //worker->handle = CreateThread(NULL, 0xffff, (LPTHREAD_START_ROUTINE)thread_main<platform::threads::thread_meta::tile_thrd_fn>, &controls, NULL, NULL);
}

u32 platform::threads::osResolveAvailableBatchProcessors()
{
    return std::thread::hardware_concurrency();
}

void platform::threads::osAssignAndWakeBatchProcessors(thread_meta::tile_thrd_fn fn,
                                                       osThread<thread_meta::TILE>* workgroup, u8 workgroup_width, u8 workgroup_height, u16 workgroup_area)
{
    for (u16 i = 0; i < workgroup_area; i++)
    {
        workgroup[i].next_task = fn;
        workgroup[i].next_task_params = { workgroup_width, workgroup_height, i };
    }
    WakeAllConditionVariable(batch_cvar);
}

void platform::threads::osAssignAndWakeWorker(thread_meta::worker_thrd_fn fn, osThread<thread_meta::WORKER>* worker)
{
    worker->next_task = fn;
    // No parameters for independant worker threads
    WakeConditionVariable((PCONDITION_VARIABLE)worker->cvar);
}

void platform::threads::osWaitForSignal(platform::threads::osAtomicInt* messenger, u32 expected_value)
{
    while (messenger->load() != expected_value)
    { /* Busy wait */ }
}

void platform::threads::osThreadGeneric::osWaitForExecution()
{
    WaitForSingleObject(handle, INFINITE);
}

void platform::threads::osTerminateBatchProcessors(osThread<thread_meta::TILE>* workgroup, u16 workgroup_area)
{
    // End thread main loops
    for (u16 i = 0; i < workgroup_area; i++)
    {
        workgroup[i].thread_openness->dec();
    }

    // Wake batched threads
    WakeAllConditionVariable(batch_cvar);

    // Wait for each thread to wind-up, then clear its critical section
    for (u16 i = 0; i < workgroup_area; i++)
    {
        workgroup[i].osWaitForExecution();
        DeleteCriticalSection((PCRITICAL_SECTION)workgroup[i].thread_guard);
    }
}

void platform::threads::osTerminateWorker(osThread<thread_meta::WORKER>* worker)
{
    worker->thread_openness->dec(); // Close the thread
    WakeConditionVariable((PCONDITION_VARIABLE)worker->cvar);
    worker->osWaitForExecution(); // Wait for it to finish in-flight work
    TerminateThread(worker->handle, 0); // Terminate the thread, with a zero exit code since it was intentional
    DeleteCriticalSection((PCRITICAL_SECTION)worker->thread_guard);
}

HWND hWnd;
BITMAPINFO bmi;
bool platform::osWindowSetup(void* hInstance, int nCmdShow, void* wndProcSig, wchar_t* wndClassName, wchar_t* wndTitle, u16 w, u16 h)
{
    // Register window class
    WNDCLASSEXW wcex;
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wcex.lpfnWndProc = (WNDPROC)wndProcSig;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = (HINSTANCE)hInstance;
    wcex.hIcon = LoadIcon((HINSTANCE)hInstance, MAKEINTRESOURCE(IDI_VOXSCULPT));
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(GetStockObject(LTGRAY_BRUSH));
    wcex.lpszMenuName = MAKEINTRESOURCEW(IDC_VOXSCULPT);
    wcex.lpszClassName = wndClassName;
    wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
    bool reg_success = RegisterClassExW(&wcex);

    // Create window
    hWnd = CreateWindowW(wndClassName, wndTitle, WS_OVERLAPPEDWINDOW,
                         CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, (HINSTANCE)hInstance, nullptr);
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
    nfo.bmiHeader.biWidth = w;
    nfo.bmiHeader.biHeight = h;
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

void* platform::osGetWindowBmi()
{
    return &bmi;
}

u64 platform::osGetWindowHandle()
{
    return reinterpret_cast<u64>(hWnd);
}

void platform::osAssertion(bool test)
{
    assert(test);
}

void platform::osDebugLog(const char* str_const)
{
    OutputDebugStringA(str_const);
}

void platform::osDebugLog(char* str, u32 len_printable)
{
    assert(len_printable <= std::strlen(str)); // Output limits cannot be higher than string lengths
    str[len_printable] = 0;
    OutputDebugStringA(str);
}

void platform::osDebugBreak()
{
    DebugBreak();
}

u64 platform::osGetCurrentTimeNanoSeconds()
{
    auto curr_t = std::chrono::high_resolution_clock::now();
    u64 epoch_time_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(curr_t.time_since_epoch()).count();
    return epoch_time_nanos;
}

double platform::osGetCurrentTimeMilliSeconds()
{
    return static_cast<double>(osGetCurrentTimeNanoSeconds()) * 1e-3;
}

double platform::osGetCurrentTimeSeconds()
{
    return static_cast<double>(osGetCurrentTimeNanoSeconds()) * 1e-9;
}

void* platform::osMalloc(u64 size, u64 alignment)
{
    return _aligned_malloc(static_cast<size_t>(size), static_cast<size_t>(alignment));
}

void platform::osFree(void* address)
{
    _aligned_free(address);
}

void platform::osClearMem(void* address, u32 length)
{
    ZeroMemory(address, length);
}