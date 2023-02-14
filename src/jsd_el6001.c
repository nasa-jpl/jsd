#include "jsd/jsd_el6001.h"

#include <assert.h>

#include "jsd/jsd.h"
#include "jsd/jsd_sdo.h"

static bool jsd_el6001_statusword_bit_is_set(const jsd_t* self, uint16_t slave_id, int bit) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(bit >= 0);
  assert(bit < JSD_EL6001_NUM_STATUS_WORD_BITS);

  return self->slave_states[slave_id].el6001.statusword & (1 << bit);  
}

static int jsd_el6001_check_for_faults(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_state_t* state  = &self->slave_states[slave_id].el6001;

  if (state->statusword != state->statusword_last)
  {
    if (jsd_el6001_statusword_bit_is_set(self, slave_id, JSD_EL6001_STATUS_WORD_FIFO_BUFFER_FULL))
    {
      ERROR("EL6001[%d]: FIFO buffer is full", slave_id);
    }

    if (jsd_el6001_statusword_bit_is_set(self, slave_id, JSD_EL6001_STATUS_WORD_PARITY_ERROR))
    {
      ERROR("EL6001[%d]: Parity error", slave_id);
    }

    if (jsd_el6001_statusword_bit_is_set(self, slave_id, JSD_EL6001_STATUS_WORD_FRAMING_ERROR))
    {
      ERROR("EL6001[%d]: Framing error", slave_id);
    }

    if (jsd_el6001_statusword_bit_is_set(self, slave_id, JSD_EL6001_STATUS_WORD_OVERRUN_ERROR))
    {
      ERROR("EL6001[%d]: Overrun error", slave_id);
    }
  }

  return 0;
}

/****************************************************
 * Public functions
 ****************************************************/

const jsd_el6001_state_t* jsd_el6001_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);

  return &self->slave_states[slave_id].el6001;
}

void jsd_el6001_read_PDO_data(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_state_t* state  = &self->slave_states[slave_id].el6001;         

  jsd_el6001_txpdo_data_t* txpdo =
      (jsd_el6001_txpdo_data_t*)self->ecx_context.slavelist[slave_id].inputs;

  // jsd_el3602_rxpdo_t* rxpdo =
  //     (jsd_el3602_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;      

  // state->control_word_terminal_last = state->control_word_terminal;
  // state->control_word = rxpdo->control_word;

  state->statusword_last = state->statusword;
  state->statusword = txpdo->statusword;
}

void jsd_el6001_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);

  // read PDOs, updates status_word
  jsd_el6001_read_PDO_data(self, slave_id);

  jsd_el6001_check_for_faults(self, slave_id);
  
}

// void jsd_el6001_write_single_channel(jsd_t* self, uint16_t slave_id,
//                                      uint8_t channel, double output) {
//   assert(self);
//   assert(self->ecx_context.slavelist[slave_id].eep_id ==
//          JSD_EL6001_PRODUCT_CODE);
// }

// void jsd_el6001_write_all_channels(jsd_t* self, uint16_t slave_id,
//                                    double output[JSD_EL6001_NUM_CHANNELS]) {
//   // for (int ch = 0; ch < JSD_EL6001_NUM_CHANNELS; ++ch) {
//   //   jsd_el6001_write_single_channel(self, slave_id, ch, output[ch]);
//   // }
// }

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el6001_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  jsd_slave_config_t* config = &self->slave_configs[slave_id];

  // No PO2SO callback for 4102 devices, so set the success flag now
  config->PO2SO_success = true;

  return true;
}
