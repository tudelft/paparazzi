/* Copyright 2020-2021 The MathWorks, Inc. */
/* Wrapper file for time.h that ensures the proper POSIX defines for MATLAB Coder */
#ifndef CODER_POSIX_TIME_H
#define CODER_POSIX_TIME_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief Represents generic timestamps
 *
 */
typedef struct coderTimespec_tag {
    double tv_sec;
    double tv_nsec;
} coderTimespec;

/**
 * @brief Represents a localized time
 *
 */
typedef struct coderTm_tag {
    long tm_nsec; /* nanoseconds */
    int tm_sec;   /* seconds */
    int tm_min;   /* minutes */
    int tm_hour;  /* hours */
    int tm_mday;  /* day of the month */
    int tm_mon;   /* month */
    int tm_year;  /* year */
    int tm_wday;  /* day of the week */
    int tm_yday;  /* day in the year */
    int tm_isdst; /* daylight saving time */
} coderTm;

/**
 * @brief Initialize state for time functions
 *
 * @param aFrequency Returned clock frequency for Windows APIs
 * @return int Status code. 0 for success. Non-zero for failure
 */
int coderInitTimeFunctions(double* const aFrequency);

/**
 * @brief Returns a timestamp from a monotonic clock
 *
 * @param aCoderTimespec The returned timestamp
 * @param aFrequency Frequency obtained from coderInitTimeFunctions
 * @return int Status code. 0 for success. Non-zero for failure
 */
int coderTimeClockGettimeMonotonic(coderTimespec* const aCoderTimespec, double aFrequency);

/**
 * @brief Sleep for a specified duration
 *
 * @param aCoderTimespec Amount of time to sleep for
 * @return int Status code. 0 for success. Non-zero for failure
 */
int coderTimeSleep(coderTimespec const * const aCoderTimespec);

/**
 * @brief Query the local time
 *
 * @param aTm Returned local time
 * @return int Status code. 0 for success. Non-zero for failure
 */
int coderLocalTime(coderTm* const aTm);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CODER_POSIX_TIME_H */
