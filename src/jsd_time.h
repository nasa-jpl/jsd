#ifndef JSD_TIME_H_
#define JSD_TIME_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/time.h>
#include <time.h>

/**
 * @brief Get the system's clock time since the Unix Epoch.
 * @return Number of seconds since Unix Epoch.
 */
static inline double jsd_time_get_time_sec() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
}

#ifdef __cplusplus
}
#endif

#endif
