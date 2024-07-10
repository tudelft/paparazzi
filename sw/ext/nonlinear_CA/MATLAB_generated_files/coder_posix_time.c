/* MATLAB Coder time APIs */
/* Copyright 2020-2022 The MathWorks, Inc. */

/* Sort out the defines early before any headers are included
 * to avoid includes with inconsistent defines
 */
#if (!defined(CODER_WINAPI)) && (defined(_WIN32))
#define CODER_WINAPI
#endif

/* Ensure we get the right _POSIX_C_SOURCE and isolate our define to just this C file */
#if !defined(CODER_WINAPI)
#undef _POSIX_C_SOURCE
#if defined(__QNX__)
/* QNX wants a newer POSIX version to make CLOCK_MONOTONIC available */
#define _POSIX_C_SOURCE 200112L
#else
#define _POSIX_C_SOURCE 199309L
#endif /* defined(__QNX__) */
#endif /* !defined(CODER_WINAPI) */

#ifdef CODER_WINAPI
#define NOMINMAX
#include <windows.h>
#endif /* CODER_WINAPI */

#include "coder_posix_time.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

/* Prototypes */
#ifndef CODER_WINAPI
/**
 * @brief Convert coderTimeSpec to POSIX timespec
 *
 */
static void coderToPOSIXTimespec(coderTimespec const * const aCoderTimespec,
                                 struct timespec* const aTimespec);

/**
 * @brief Convert POSIX timespec to coderTimeSpec
 *
 */
static void posixToCoderTimespec(const struct timespec* const aTimespec,
                                 coderTimespec* const aCoderTimespec);

static void coderToPOSIXTimespec(coderTimespec const * const aCoderTimespec,
                                 struct timespec* const aTimespec) {
    aTimespec->tv_sec = (time_t)(aCoderTimespec->tv_sec);
    aTimespec->tv_nsec = (long)(aCoderTimespec->tv_nsec);
}

static void posixToCoderTimespec(const struct timespec* const aTimespec,
                                 coderTimespec* const aCoderTimespec) {
    aCoderTimespec->tv_sec = (double)(aTimespec->tv_sec);
    aCoderTimespec->tv_nsec = (double)(aTimespec->tv_nsec);
}
#endif /* CODER_WINAPI */

int coderInitTimeFunctions(double* const aFrequency) {
#ifdef CODER_WINAPI
    LARGE_INTEGER freqL;
    int status;
    status = QueryPerformanceFrequency(&freqL);
    *aFrequency = (double)freqL.QuadPart;
    /* Invert status since in Windows, 0 means failure */    
    return status == 0;
#else
    *aFrequency = 0.0;
    return 0;
#endif
}

int coderTimeClockGettimeMonotonic(coderTimespec* const aCoderTimespec, double aFrequency) {
#ifdef CODER_WINAPI
    static const double BILLION = 1e9;
    LARGE_INTEGER timeL;
    double timeDouble, timeRemainder, seconds, nanoseconds;
    BOOL status = QueryPerformanceCounter(&timeL);
    timeDouble = (double)timeL.QuadPart;
    timeRemainder = fmod(timeDouble, aFrequency);
    seconds = (timeDouble - timeRemainder) / aFrequency;
    nanoseconds = (timeRemainder * BILLION) / aFrequency;
    aCoderTimespec->tv_sec = seconds;
    aCoderTimespec->tv_nsec = nanoseconds;

    /* Invert status since in Windows, 0 means failure */
    return status == 0;
#else
    struct timespec tspec;
    int status;
    (void) aFrequency;
    status = clock_gettime(CLOCK_MONOTONIC, &tspec);
    posixToCoderTimespec(&tspec, aCoderTimespec);
    return status;
#endif /* defined(_WIN32) || defined(WIN32) */
}

int coderTimeSleep(coderTimespec const * const aCoderTimespec) {
#ifdef CODER_WINAPI
    /* WIndows */
    static const double MILLI = 1e3;
    static const double BILLION = 1e9;
    double delayMilli, delay;
    DWORD delayUint32;
    delayMilli = floor(aCoderTimespec->tv_nsec / (BILLION / MILLI));
    delay = MILLI * aCoderTimespec->tv_sec + delayMilli;
    delayUint32 = (DWORD)ceil(delay);
    Sleep(delayUint32);
    return 0;
#else
    /* POSIX */
    struct timespec ts;
    coderToPOSIXTimespec(aCoderTimespec, &ts);
    return nanosleep(&ts, NULL);
#endif
}

int coderLocalTime(coderTm* const aTm) {
#ifdef CODER_WINAPI
    /* WIndows */
    SYSTEMTIME sysTime;
    TIME_ZONE_INFORMATION tzInfo;
    DWORD dstInfo;
    GetLocalTime(&sysTime);
    aTm->tm_nsec = ((long)sysTime.wMilliseconds) * 1000000;
    aTm->tm_sec = sysTime.wSecond;
    aTm->tm_min = sysTime.wMinute;
    aTm->tm_hour = sysTime.wHour;
    aTm->tm_mday = sysTime.wDay;
    aTm->tm_mon = sysTime.wMonth;
    aTm->tm_year = sysTime.wYear;
    aTm->tm_wday = 0;
    aTm->tm_yday = 0;

    dstInfo = GetTimeZoneInformation(&tzInfo);
    aTm->tm_isdst = dstInfo == TIME_ZONE_ID_DAYLIGHT;
    return 0;
#else
    /* POSIX */
    struct timespec ts;
    struct tm* tmVal;
    int status;
    const int YEAR_OFFSET = 1900; /* +1900 for human year */
    const int MONTH_OFFSET = 1; /* +1 for 1-based month */
    status = clock_gettime(CLOCK_REALTIME, &ts);
    tmVal = localtime(&ts.tv_sec);
    aTm->tm_nsec = ts.tv_nsec;
    aTm->tm_sec = tmVal->tm_sec;
    aTm->tm_min = tmVal->tm_min;
    aTm->tm_hour = tmVal->tm_hour;
    aTm->tm_mday = tmVal->tm_mday;
    aTm->tm_mon = tmVal->tm_mon + MONTH_OFFSET;
    aTm->tm_year = tmVal->tm_year + YEAR_OFFSET;
    aTm->tm_wday = tmVal->tm_wday;
    aTm->tm_yday = tmVal->tm_yday;
    aTm->tm_isdst = tmVal->tm_isdst;

    return status;
#endif
}
