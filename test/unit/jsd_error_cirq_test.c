#include "jsd/jsd_error_cirq.h"
#include "jsd/jsd_print.h"
#include <assert.h>

int main() {

  jsd_error_cirq_t q;

  MSG("Initializing jsd_error_cirq_t");

  // init it and confirm empty
  jsd_error_cirq_init(&q, "my_device_test_elist");
  assert(jsd_error_cirq_is_empty(&q));

  ec_errort ex_error = {};

  unsigned int push_cnt = 0;

  // fill it
  unsigned int i;
  for(i = 0; i <= (JSD_ERROR_CIRQ_LEN-1) ; i++){
    ex_error.Slave = push_cnt++;
    MSG("Adding %d", i);
    assert(jsd_error_cirq_push(&q, ex_error));
    assert(!jsd_error_cirq_is_empty(&q));
  }

  // empty it
  for(i = 0; i <= (JSD_ERROR_CIRQ_LEN-1); i++){
    assert(!jsd_error_cirq_is_empty(&q));
    ex_error = jsd_error_cirq_pop(&q);
    MSG("Popping %d", ex_error.Slave);
    assert( ex_error.Slave == i);
  }

  // empty it and confirm empty
  ex_error = jsd_error_cirq_pop(&q);
  assert(ex_error.Slave == 0);
  assert(jsd_error_cirq_is_empty(&q));

  // Now check for overflow
  for(i = 0; i < JSD_ERROR_CIRQ_LEN; i++){
    ex_error.Slave = push_cnt++;
    assert(jsd_error_cirq_push(&q, ex_error));
    assert(!jsd_error_cirq_is_empty(&q));
  }

  assert(!jsd_error_cirq_push(&q, ex_error));
  ex_error = jsd_error_cirq_pop(&q);

  MSG("Popping %d", ex_error.Slave);
  assert( ex_error.Slave == (JSD_ERROR_CIRQ_LEN+1) );

  MSG("Done");

  return 0;
}
