#ifndef JSD_PRINT_H_
#define JSD_PRINT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cfw/cfw_print.h"

#include <stdio.h>

#ifdef DEBUG
#define MSG_DEBUG(M, ...) \
  cfw_error_print(CFW_DEBUG, __FILE__, __LINE__, M, ##__VA_ARGS__)
#else
#define MSG_DEBUG(M, ...)
#endif

#define MSG(M, ...) \
  cfw_error_print(CFW_INFO, __FILE__, __LINE__, M, ##__VA_ARGS__)

#define WARNING(M, ...) \
  cfw_error_print(CFW_WARN, __FILE__, __LINE__, M, ##__VA_ARGS__)

#define ERROR(M, ...) \
  cfw_error_print(CFW_ERROR, __FILE__, __LINE__, M, ##__VA_ARGS__)

#define SUCCESS(M, ...) \
  cfw_error_print(CFW_INFO, __FILE__, __LINE__, M, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif
