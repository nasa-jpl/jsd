#ifndef JSD_ERROR_CIRQ_H_
#define JSD_ERROR_CIRQ_H_

#include "ethercat.h"

#include "jsd/jsd_include_defs.h"

#include <stdbool.h>

#define JSD_ERROR_CIRQ_LEN (8)

typedef struct {
  ec_errort       buffer[JSD_ERROR_CIRQ_LEN];
  uint16_t        r;
  uint16_t        w;
  char            name[JSD_NAME_LEN];
  pthread_mutex_t mutex;
} jsd_error_cirq_t;

void jsd_error_cirq_init(jsd_error_cirq_t* self, const char* name);

ec_errort jsd_error_cirq_pop(jsd_error_cirq_t* self);

bool jsd_error_cirq_push(jsd_error_cirq_t* self, ec_errort error);

bool jsd_error_cirq_is_empty(jsd_error_cirq_t* self);

#endif
