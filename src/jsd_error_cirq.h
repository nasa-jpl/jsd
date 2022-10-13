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

/**
 * @brief Initializes a new error circular queue with name
 *
 * @param self error circular queue context
 * @param name name of the queue for error tracking
 * @return void
 */
void jsd_error_cirq_init(jsd_error_cirq_t* self, const char* name);

/**
 * @brief Checks if the queue is empty
 *
 * real-time safe, thread-safe
 *
 * Note: You probably do not need to call this directly. Instead,
 *   call the pop() function and check the return status there. 
 *
 * @param self error circular queue context
 * @return true if empty
 */
bool jsd_error_cirq_is_empty(jsd_error_cirq_t* self);

/**
 * @brief Pops a value from the error circular queue
 *
 * real-time safe, thread-safe
 *
 * @param self error circular queue context
 * @param error_out the new error, NULL if queue was empty
 * @return true if error_out contains good data
 */
bool jsd_error_cirq_pop(jsd_error_cirq_t* self, ec_errort* error_out);

/**
 * @brief Pushes a new error to the circular queue
 *
 * real-time safe, thread-safe
 *
 * @param self error circular queue context
 * @param error the new error to be added to queue
 * @return void
 */
void jsd_error_cirq_push(jsd_error_cirq_t* self, ec_errort error);


#endif
