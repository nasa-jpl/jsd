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
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000;
}

#ifdef __cplusplus
}
#endif

#endif
