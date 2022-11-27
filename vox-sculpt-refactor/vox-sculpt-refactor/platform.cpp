
#include <thread>
#include <sdkddkver.h>
#include <Windows.h>
#undef min
#undef max
#include "resource.h"
#include <assert.h>
#include <chrono>
#include "platform.h"

// Bucket used for (singlethreaded) formatted text output, duration with maximum 1024 characters supported
// (most lines should be much shorter)
static constexpr uint16_t txt_buf_len = 1024;
static char txt_buf[txt_buf_len];

int32_t platform::formatting_internal(char** output, const char* txt, ...)
{
    char txt_staging[txt_buf_len] = {};
    va_list args;
    va_start(args, txt_staging);
    vprintf(txt_staging, args);
    va_end(args);

    *output = txt_buf;
    return sprintf_s(txt_buf, txt_buf_len, txt_staging);
}

uint32_t platform::osResolveAvailableThreads()
{
    // Try to leave at least two cores spare for background processing
    return std::thread::hardware_concurrency() - 2;
}

void platform::osAssertion(bool test)
{
    assert(test);
}

void platform::osDebugLog(const char* str_const)
{
    OutputDebugStringA(str_const);
}

void platform::osDebugLog(char* str, uint32_t len_printable)
{
    const uint16_t len = std::strlen(str) + 1; // sprintf skips the null terminator, so account for that here
    assert(len_printable <= len); // Output limits cannot be higher than string lengths
    str[len_printable] = 0; // Add the missing null terminator back here
    OutputDebugStringA(str);
}

void platform::osDebugBreak()
{
    DebugBreak();
}

uint64_t platform::osGetCurrentTimeNanoSeconds()
{
    auto curr_t = std::chrono::high_resolution_clock::now();
    uint64_t epoch_time_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(curr_t.time_since_epoch()).count();
    return epoch_time_nanos;
}

double platform::osGetCurrentTimeMilliSeconds()
{
    return static_cast<double>(osGetCurrentTimeNanoSeconds()) * 1e-6;
}

double platform::osGetCurrentTimeSeconds()
{
    return static_cast<double>(osGetCurrentTimeNanoSeconds()) * 1e-9;
}

void* platform::osMalloc(uint64_t size, uint64_t alignment)
{
    return _aligned_malloc(static_cast<size_t>(size), static_cast<size_t>(alignment));
}

void platform::osFree(void* address)
{
    _aligned_free(address);
}

void platform::osClearMem(void* address, uint32_t length)
{
    ZeroMemory(address, length);
}

void platform::osSetMem(void* address, uint8_t byte_pattern, uint32_t length)
{
    memset(address, byte_pattern, length);
}

void platform::osCpyMem(void* dst, void* src, uint64_t size)
{
    memcpy(dst, src, size);
}

bool keys[(uint32_t)platform::VOX_SCULPT_KEYS::NUM_SUPPORTED_KEYS] = { };
void platform::osKeyDown(platform::VOX_SCULPT_KEYS keyID)
{
    // If keyID is >= NUM_SUPPORTED_KEYS, then either a bug happened or the current key should be added to VOX_SCULPT_KEYS
    assert(keyID < platform::VOX_SCULPT_KEYS::NUM_SUPPORTED_KEYS);

    // Toggle key state
    keys[(uint32_t)keyID] = true;
}
void platform::osKeyUp(platform::VOX_SCULPT_KEYS keyID)
{
    // If keyID is >= NUM_SUPPORTED_KEYS, then either a bug happened or the current key should be added to VOX_SCULPT_KEYS
    assert(keyID < platform::VOX_SCULPT_KEYS::NUM_SUPPORTED_KEYS);

    // Toggle key state
    keys[(uint32_t)keyID] = false;
}

bool platform::osTestKey(platform::VOX_SCULPT_KEYS key)
{
    return keys[(uint32_t)key];
}