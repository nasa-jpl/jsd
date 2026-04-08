#ifndef JSD_PRINT_H_
#define JSD_PRINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <jsd/jsd_time.h>
#include <stdio.h>

#if defined(__ZEPHYR__)
/* On Zephyr: suppress MSG_DEBUG entirely and use integer millisecond
 * timestamps for all other macros to avoid the broken %lf formatter
 * which adds hundreds of microseconds of overhead per call. */
#define MSG_DEBUG(M, ...) do { } while (0)
#define MSG(M, ...)      do { } while (0)
#define WARNING(M, ...)  do { } while (0)
#define SUCCESS(M, ...)  do { } while (0)

#define JSD_PRINT_TS_MS ((long)(jsd_time_get_time_sec() * 1000.0))

#define ERROR(M, ...)                                                 \
  fprintf(stderr, "[ ERROR ] [%ld ms] (%s:%d) " M "\n",              \
          JSD_PRINT_TS_MS, __FILE__, __LINE__, ##__VA_ARGS__)

#else /* not Zephyr */

#ifdef DEBUG
#define MSG_DEBUG(M, ...)                                                     \
  fprintf(stderr, "[ DEBUG ] [%lf] (%s:%d) " M "\n", jsd_time_get_time_sec(), \
          __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define MSG_DEBUG(M, ...)
#endif

#define MSG(M, ...)                                                           \
  fprintf(stderr, "[ INFO  ] [%lf] (%s:%d) " M "\n", jsd_time_get_time_sec(), \
          __FILE__, __LINE__, ##__VA_ARGS__)

#define WARNING(M, ...)                                               \
  fprintf(stderr, "\033[1;33m[ WARN  ] [%lf] (%s:%d) " M "\033[0m\n", \
          jsd_time_get_time_sec(), __FILE__, __LINE__, ##__VA_ARGS__)

#define ERROR(M, ...)                                                 \
  fprintf(stderr, "\033[1;31m[ ERROR ] [%lf] (%s:%d) " M "\033[0m\n", \
          jsd_time_get_time_sec(), __FILE__, __LINE__, ##__VA_ARGS__)

#define SUCCESS(M, ...)                                               \
  fprintf(stderr, "\033[1;32m[SUCCESS] [%lf] (%s:%d) " M "\033[0m\n", \
          jsd_time_get_time_sec(), __FILE__, __LINE__, ##__VA_ARGS__)

#endif /* __ZEPHYR__ */

#ifdef __cplusplus
}
#endif

#endif