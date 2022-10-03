#include "jsd/jsd_error_cirq.h"
#include "jsd/jsd_print.h"

#include <assert.h>
#include <string.h>

#include "jsd/jsd_print.h"

void jsd_error_cirq_init(jsd_error_cirq_t* self, const char* name){
  assert(self);
  self->r     = 0;
  self->w     = 0;
  self->mutex = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;

  strncpy(self->name, name, JSD_NAME_LEN);
}

// non-locking
static ec_errort pop_error(jsd_error_cirq_t* self) {
  ec_errort new_error;

  MSG_DEBUG("[%s] r=%u w=%u", self->name, self->r, self->w);
  new_error = self->buffer[self->r % JSD_ERROR_CIRQ_LEN];
  self->r++;

  return new_error;
}

ec_errort jsd_error_cirq_pop(jsd_error_cirq_t* self) {
  assert(self);
  ec_errort new_error = {};
  if (jsd_error_cirq_is_empty(self)) {
    WARNING("Popping [%s] when empty", self->name);
    return new_error;
  }

  pthread_mutex_lock(&self->mutex);
  new_error = pop_error(self);
  pthread_mutex_unlock(&self->mutex);

  return new_error;
}

// non-locking
static bool push_error(jsd_error_cirq_t* self, ec_errort error) {
  bool status = true;

  self->buffer[self->w % JSD_ERROR_CIRQ_LEN] = error;
  self->w++;
  if ((self->w - self->r) > JSD_ERROR_CIRQ_LEN) {
    self->r++;
    WARNING("[%s] is overflowing: r=%u w=%u", self->name, self->r, self->w);
    status = false;
  }

  return status;
}

bool jsd_error_cirq_push(jsd_error_cirq_t* self, ec_errort error) {
  assert(self);
  bool status;
  pthread_mutex_lock(&self->mutex);
  status = push_error(self, error);
  pthread_mutex_unlock(&self->mutex);
  return status;
}

bool jsd_error_cirq_is_empty(jsd_error_cirq_t* self) {
  assert(self);
  bool val;
  pthread_mutex_lock(&self->mutex);
  val = (self->r == self->w);
  pthread_mutex_unlock(&self->mutex);
  return val;
}
