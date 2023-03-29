#ifndef JSD_PRINT_H_
#define JSD_PRINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <jsd/jsd_time.h>
#include <stdio.h>

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

#ifdef __cplusplus
}
#endif

#endif