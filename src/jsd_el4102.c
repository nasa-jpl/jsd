#include "jsd/jsd_el4102.h"

#include <assert.h>

#include "jsd/jsd_sdo.h"

/****************************************************
 * Public functions
 ****************************************************/

const jsd_el4102_state_t* jsd_el4102_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL4102_PRODUCT_CODE);

  return &self->slave_states[slave_id].el4102;
}

void jsd_el4102_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL4102_PRODUCT_CODE);

  jsd_el4102_rxpdo_t* rxpdo =
      (jsd_el4102_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;

  for (int ch = 0; ch < JSD_EL4102_NUM_CHANNELS; ++ch) {
    rxpdo->channel[ch].value =
        self->slave_states[slave_id].el4102.dac_output[ch];
  }

  jsd_async_sdo_process_response(self, slave_id);
}

void jsd_el4102_write_single_channel(jsd_t* self, uint16_t slave_id,
                                     uint8_t channel, int16_t output) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL4102_PRODUCT_CODE);

  self->slave_states[slave_id].el4102.dac_output[channel] = output;
}

void jsd_el4102_write_all_channels(jsd_t* self, uint16_t slave_id,
                                   int16_t output[JSD_EL4102_NUM_CHANNELS]) {
  for (int ch = 0; ch < JSD_EL4102_NUM_CHANNELS; ++ch) {
    jsd_el4102_write_single_channel(self, slave_id, ch, output[ch]);
  }
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el4102_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL4102_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  jsd_slave_config_t* config = &self->slave_configs[slave_id];

  // No PO2SO callback for 4102 devices, so set the success flag now
  config->PO2SO_success = true;

  return true;
}