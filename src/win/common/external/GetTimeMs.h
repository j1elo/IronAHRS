#ifndef GetTimeMs_H_
#define GetTimeMs_H_

#ifdef _WIN32
    #include <Windows.h>
#else
    #include <sys/time.h>
    #include <ctime>
#endif

#include <cstdint>

/**
 * Return the amount of milliseconds elapsed since the UNIX epoch.
 *
 * It will work like time(NULL), but will return the number of milliseconds
 * instead of seconds from the UNIX epoch on both Windows and Linux.
 *
 * On Windows, time is obtained from GetSystemTimeAsFileTime(),
 * which has limited resolution and depends on the system:
 * - Windows XP and earlier: resolution is 15.625 msecs or 10 msecs depending on hardware.
 * - Windows 7: resolution seems to be sub-millisecond.
 *
 * On Linux, gettimeofday()is used, which is implementation dependent,
 * but usually has 15 milliseconds resolution as well.
 *
 * Thread safe.
 */
uint64_t GetTimeMs();

#ifdef _WIN32
void StartWinTimeCounter();
double GetWinTimeMs();
#endif

#endif // GetTimeMs_H_
