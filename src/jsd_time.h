#ifndef JSD_TIME_H_
#define JSD_TIME_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/time.h>
#include <time.h>

#include "ethercattype.h"

/**
 * @brief Get the system's clock time since the Unix Epoch.
 * @return Number of seconds since Unix Epoch.
 */
static inline double jsd_time_get_time_sec() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000;
}

/**
 * @brief Get monotonic time since unspecified fixed point. This function is
 * used to compute elapsed time.
 * @return Number of seconds since fixed point.
 */
static inline double jsd_time_get_mono_time_sec() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000;
}

/**
 * @brief Convert SOEM's time type to seconds.
 * @return Seconds representation of SOEM's time type object.
 */
static inline double ectime_to_sec(ec_timet t) {
  return (double)t.sec + (double)(t.usec) * 1.0e-6;
}

#ifdef __cplusplus
}
#endif

#endif
