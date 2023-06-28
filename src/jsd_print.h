#ifndef JSD_PRINT_H_
#define JSD_PRINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_time.h"
#include "cfw/cfw_log.h"

#include <stdio.h>

#if CFW_LOG_VERBOSITY >= 1
#define ERROR(M, ...) \
  cfw_log_print(CFW_ERROR, __FILE__, __LINE__, M, ##__VA_ARGS__)
#else
#define ERROR(M, ...) \
  do {                \
  } while (0)
#endif

#if CFW_LOG_VERBOSITY >= 2
#define WARNING(M, ...) \
  cfw_log_print(CFW_WARN, __FILE__, __LINE__, M, ##__VA_ARGS__)
#else
#define WARNING(M, ...) \
  do {                \
  } while (0)
#endif

#if CFW_LOG_VERBOSITY >=3
#define MSG(M, ...) \
  cfw_log_print(CFW_INFO, __FILE__, __LINE__, M, ##__VA_ARGS__)
#define SUCCESS(M, ...) \
  cfw_log_print(CFW_INFO, __FILE__, __LINE__, M, ##__VA_ARGS__)
#else
#define MSG(M, ...) \
  do {              \
  } while (0)
#define SUCCESS(M, ...) \
  do {              \
  } while (0)
#endif

#if CFW_LOG_VERBOSITY >= 4
#define MSG_DEBUG(M, ...) \
  cfw_log_print(CFW_DEBUG, __FILE__, __LINE__, M, ##__VA_ARGS__)
#else
#define MSG_DEBUG(M, ...) \
  do {              \
  } while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif
