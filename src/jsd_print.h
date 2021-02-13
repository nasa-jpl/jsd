#ifndef JSD_PRINT_H_
#define JSD_PRINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#ifdef DEBUG
#define MSG_DEBUG(M, ...) \
  fprintf(stderr, "[ DEBUG ](%s:%d) " M "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define MSG_DEBUG(M, ...)
#endif

#define MSG(M, ...) \
  fprintf(stderr, "[ INFO  ](%s:%d) " M "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define WARNING(M, ...)                                                  \
  fprintf(stderr, "\033[1;33m[ WARN  ](%s:%d) " M "\033[0m\n", __FILE__, \
          __LINE__, ##__VA_ARGS__)

#define ERROR(M, ...)                                                    \
  fprintf(stderr, "\033[1;31m[ ERROR ](%s:%d) " M "\033[0m\n", __FILE__, \
          __LINE__, ##__VA_ARGS__)

#define SUCCESS(M, ...)                                                  \
  fprintf(stderr, "\033[1;32m[SUCCESS](%s:%d) " M "\033[0m\n", __FILE__, \
          __LINE__, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif
