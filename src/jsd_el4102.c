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
                                     uint8_t channel, double output) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL4102_PRODUCT_CODE);

  // Clamp requested command within [0-10] V.
  const double t = output < 0.0 ? 0.0 : output;
  output         = t > 10.0 ? 10.0 : t;

  // EL4102 has a 0-10V range and a 16-bit integer DAC value. The available
  // discrete values for that range are 0x0000 - 0x7FFF:
  // 1/((10-0)/(2^16/2-1))=3276.7 discrete levels/V.

  self->slave_states[slave_id].el4102.voltage_output[channel] = output;
  self->slave_states[slave_id].el4102.dac_output[channel] =
      (int16_t)(output * 3276.7);
}

void jsd_el4102_write_all_channels(jsd_t* self, uint16_t slave_id,
                                   double output[JSD_EL4102_NUM_CHANNELS]) {
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