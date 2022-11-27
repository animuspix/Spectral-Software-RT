#pragma once

#include <type_traits>
#include <tuple>
#include <stdio.h>

class platform
{
public:
    // Core formatting interfaces for debug text
    ////////////////////////////////////////////
    static int32_t formatting_internal(char** output, const char* txt, ...);

    template<typename...logging_types>
    static char* txt_formatter(const char* txt, int32_t* chars_written, logging_types...formatted_data)
    {
        char* output;
        *chars_written = formatting_internal(&output, txt, formatted_data);
        return output;
    }

    // Key mappings & basic keyboard interface
    //////////////////////////////////////////

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

    static void osKeyDown(VOX_SCULPT_KEYS keyID);
    static void osKeyUp(VOX_SCULPT_KEYS keyID);
    static bool osTestKey(VOX_SCULPT_KEYS key);

    // More debug features
    //////////////////////

    static void osAssertion(bool test);
    static void osDebugLog(const char* str_const);
    static void osDebugLog(char* str, uint32_t len_printable);
    //static void osDebugLog(const char* str_const, uint32_t len_printable)
    //{
    //    //strcpy_s(fmt_txt, fmt_txt_len, str_const);
    //    //osDebugLog(fmt_dbg_txt_buf, len_printable);
    //}

    template<typename...logging_types>
    static void osDebugLogFmt(const char* str, logging_types...formatted_data)
    {
        int32_t written_len = 0;
        char* txt = txt_formatter<logging_types...>(str, &written_len, formatted_data...);
        osDebugLog(txt, static_cast<uint32_t>(written_len));
    }

    static void osDebugBreak();

    // Timing methods
    /////////////////

    static uint64_t osGetCurrentTimeNanoSeconds();
    static double osGetCurrentTimeMilliSeconds();
    static double osGetCurrentTimeSeconds();

    // Memory methods
    /////////////////

    static void* osMalloc(uint64_t size, uint64_t alignment);
    static void osFree(void*);
    static void osClearMem(void* address, uint32_t length);
    static void osSetMem(void* address, uint8_t byte_pattern, uint32_t length);
    static void osCpyMem(void* dst, void* src, uint64_t size);

    // Resolve a useeable threacount for the current hardware
    static uint32_t osResolveAvailableThreads();
};