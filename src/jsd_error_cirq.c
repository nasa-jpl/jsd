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
static bool _ecirq_is_empty(jsd_error_cirq_t* self){
  assert(self);

  return (self->r == self->w);
}

bool jsd_error_cirq_is_empty(jsd_error_cirq_t* self) {
  assert(self);

  bool val;
  pthread_mutex_lock(&self->mutex);
  val = _ecirq_is_empty(self);
  pthread_mutex_unlock(&self->mutex);
  return val;
}

// non-locking
static bool _ecirq_pop(jsd_error_cirq_t* self, ec_errort *new_error) {
  assert(self);
  assert(new_error);

  bool retval = false;

  if(_ecirq_is_empty(self)){
    new_error = NULL;
  }else{
    *new_error = self->buffer[self->r % JSD_ERROR_CIRQ_LEN];
    self->r++;
    retval = true;
  }
  return retval;
}

bool jsd_error_cirq_pop(jsd_error_cirq_t* self, ec_errort *new_error) {
  assert(self);
  assert(new_error);

  pthread_mutex_lock(&self->mutex);
  bool is_valid  = _ecirq_pop(self, new_error);
  pthread_mutex_unlock(&self->mutex);

  return is_valid;
}

// non-locking
static void _ecirq_push(jsd_error_cirq_t* self, ec_errort error) {
  assert(self);

  self->buffer[self->w % JSD_ERROR_CIRQ_LEN] = error;
  self->w++;
  if ((self->w - self->r) > JSD_ERROR_CIRQ_LEN) {
    self->r++;
    MSG_DEBUG("error cirq (%s) is overflowing: r=%u w=%u", self->name, self->r, self->w);
  }
}

void jsd_error_cirq_push(jsd_error_cirq_t* self, ec_errort error) {
  assert(self);

  pthread_mutex_lock(&self->mutex);
  _ecirq_push(self, error);
  pthread_mutex_unlock(&self->mutex);
}

