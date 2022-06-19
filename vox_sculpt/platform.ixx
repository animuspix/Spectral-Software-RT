export module platform;

import vox_ints;
import <type_traits>;
import <tuple>;
import <stdio.h>;

// Thread workload prototypes
// Different types of system thread can have different types of work
// Declared up here to avoid implicitly exporting these to other modules
// (atm exporting members but not the namespace will make the members unreachable - I think I tried that before)
void(*worker_fn_prototype)(); // Workers
void(*tile_fn_prototype)(u16, u16, u16); // Work-group width, work-group height, work-group index


// Internal text formatting interface; separated out to keep implementation details away from our export block below
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Bucket used for (singlethreaded) formatted text output, duration with maximum 1024 characters supported
// (most lines should be much shorter)
constexpr u16 txt_buf_len = 1024;
char txt_buf[txt_buf_len];

// Core formatting interface for debug text
template<typename...logging_types>
char* txt_formatter(const char* txt, i32* chars_written, logging_types...formatted_data)
{
    *chars_written = sprintf_s(txt_buf, txt_buf_len, txt, formatted_data...);
    return txt_buf;
}

export namespace platform
{
    // Foward-declare _everything_ to keep implementations away from modules
    namespace threads
    {
        // Memory for these _must_ come from the heap, implicitly or explicitly
        struct osAtomicInt
        {
            // Praying that SO is right and I don't need locks for volatile 32-bit values
            //u64 lock_data; // Lock object controlling load & store accesses which can't be gated by [Interlocked[x]] operations
            //               // (memory formatted by calling [InitializeSRWLock()] on Windows)
            private:
                long core_data = 0; // Core atomic data, to guarantee that our actual atomic integers are immediately behind [messenger] in memory;
                                          // code should never directly modify this member (modifications through [messenger] are totes fine & good)
                volatile long* messenger; // Kinda gross, makes win32 happy though
            public:
                void init();
                void inc();
                void dec();
                long load();
                void store(long value);
        };

        // Architectural details for the threading system used by vox_sculpt
        // Considering moving these to their own module
        // (especially if I eventually decide to support graphics accelerators)
        namespace thread_meta
        {
            // Types of async thread supported
            enum THREAD_TYPES
            {
                WORKER, // A thread associated with a job-stream, or a single task
                TILE // A tile thread, designed for batch processing within a workgroup
            };

            // Work-item prototypes for each thread-type
            typedef decltype(worker_fn_prototype) worker_thrd_fn;
            typedef decltype(tile_fn_prototype) tile_thrd_fn;

            // Parameter layouts for each thread-type (worker threads take no parameters for now)
            struct worker_params {};
            struct tile_params
            {
                u8 workgroup_width;
                u8 workgroup_height;
                u16 workgroup_index;
            };
        };

        enum thread_state
        {
            WORKING = 0x00000000,
            SLEEPING = 0xffffffff
        };

        class osThreadGeneric
        {
            public:
            osThreadGeneric(): initialized(false),
                               thread_guard(nullptr),
                               thread_openness(nullptr) {}; // Generic thread types cannot be freely created or destroyed, only upcasted from regular thread types (to allow for flexible code without
                                                            // explicit templates everywhere)
            ~osThreadGeneric(){};
            void* handle; // Associated with a Win32 HANDLE in per-platform code
            osAtomicInt* messaging; // Four bytes of atomic status, used for communicating with main/other threads
            osAtomicInt* state; // Four bytes of thread system state, either 0x000000000 (running a task/working), 0xffffffff (between tasks/sleeping), or an error code (every other value, none defined yet)
                                // Should be read by main and written by [this], except on thread launch

            // Checks if the specified thread has finished, and returns immediately if it hasn't
            //void osWaitNonblocking(osThreadGeneric* thread);

            // Directly calls [WaitForSingleObject] on the target thread
            void osWaitForExecution();

            // Test if a thread has been initialized already
            bool osCheckInitialized() { return initialized; }

            // Batch processors (TILEs) use a single statically-declared cvar to control execution, and a shared one for independant threads (WORKERs)
            void* cvar;

            // Implemented as a critical section (kind-of a user-space mutex) under Windows
            void* thread_guard;

            // Zero for closed, one for open
            // This enables efficiently terminating individual threads, while allowing regular stack unwinding & memory reclamation
            osAtomicInt* thread_openness;

            protected:
            bool initialized;
        };

        template<thread_meta::THREAD_TYPES type>
        class osThread : public osThreadGeneric
        {
            public:
                osThread() {};

                //typedef std::conditional<type == WORKER, decltype(worker_fn_prototype), decltype(tile_fn_prototype)>::type work_type; // std::conditional seems to be buggy with modules, using tuples for now
                std::tuple<thread_meta::worker_thrd_fn, thread_meta::tile_thrd_fn> work_types;
                typedef std::remove_reference<decltype(std::get<type == thread_meta::WORKER ? 0 : 1>(work_types))>::type work_type;
                work_type next_task; // Pointer to next workload to execute (no jobs for now, so just one task/thread)

                std::tuple<thread_meta::worker_params, thread_meta::tile_params> work_paramsets;
                typedef std::remove_reference<decltype(std::get<type == thread_meta::WORKER ? 0 : 1>(work_paramsets))>::type work_params;
                work_params next_task_params; // Cached parameters for each task invocation
        };

        // Find tile count for the current cpu (at runtime)
        // We assume that we flood the cpu with tile work, and any independant worker threads are timesliced in (since they'll most likely be i/o tasks that we
        // don't super mind running behind slightly)
        u32 osResolveAvailableBatchProcessors();

        // Update arguments for batched threads, then wake them
        void osAssignAndWakeBatchProcessors(thread_meta::tile_thrd_fn fn,
                                            osThread<thread_meta::TILE>* workgroup, u8 workgroup_width, u8 workgroup_height, u16 workgroup_area);

        // Update arguments for the given worker thread, then wake it
        void osAssignAndWakeWorker(thread_meta::worker_thrd_fn fn, osThread<thread_meta::WORKER>* worker);

        // Create & initialize batch processors
        void osInitBatchProcessors(osThread<thread_meta::TILE>* workgroup, u16 workgroup_area);

        // Create & initialize a single worker thread
        void osInitWorker(osThread<thread_meta::WORKER>* worker);

        // Wake & terminate batched threads
        void osTerminateBatchProcessors(osThread<thread_meta::TILE>* workgroup, u16 workgroup_area);

        // Wake & terminate a worker thread
        void osTerminateWorker(osThread<thread_meta::WORKER>* worker);

        // Waits for the specified atomic to reach a certain value
        void osWaitForSignal(osAtomicInt* messenger, u32 expected_value);

        // Get the ID associated with the calling thread on the current platform
        u32 osGetThreadId();
    };
    enum class VOX_SCULPT_KEYS
    {
        KEY_ESCAPE,
        KEY_LEFT_ARROW,
        KEY_RIGHT_ARROW,
        KEY_UP_ARROW,
        KEY_DOWN_ARROW,
        KEY_Z,
        KEY_LCTRL,
        NUM_SUPPORTED_KEYS
    };
    void osKeyDown(VOX_SCULPT_KEYS keyID);
    void osKeyUp(VOX_SCULPT_KEYS keyID);
    bool osTestKey(VOX_SCULPT_KEYS key);
    bool osWindowSetup(void* hInstance, int nCmdShow, void* wndProcSig, wchar_t* wndClassName, wchar_t* wndTitle, u16 w, u16 h);
    void* osGetWindowBmi();
    u64 osGetWindowHandle();
    void osAssertion(bool test);
    void osDebugLog(const char* str_const);
    void osDebugLog(char* str, u32 len_printable);
    void osDebugLog(const char* str_const, u32 len_printable)
    {
        //strcpy_s(fmt_txt, fmt_txt_len, str_const);
        //osDebugLog(fmt_dbg_txt_buf, len_printable);
    }
    template<typename...logging_types>
    void osDebugLogFmt(const char* str, logging_types...formatted_data)
    {
        i32 written_len = 0;
        char* txt = txt_formatter<logging_types...>(str, &written_len, formatted_data...);
        osDebugLog(txt, static_cast<u32>(written_len));
    }
    void osDebugBreak();
    u64 osGetCurrentTimeNanoSeconds();
    double osGetCurrentTimeMilliSeconds();
    double osGetCurrentTimeSeconds();
    void* osMalloc(u64 size, u64 alignment);
    void osFree(void*);
    void osClearMem(void* address, u32 length);
    void osSetMem(void* address, u8 byte_pattern, u32 length);
    void osCpyMem(void* dst, void* src, u64 size);
};