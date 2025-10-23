#include "jsd/jsd_el2828.h"

#include <assert.h>

#include "jsd/jsd_sdo.h"

/****************************************************
 * Public functions
 ****************************************************/

/**
 * @brief RxPDO struct used to set device command data in SOEM IOmap
 */
typedef struct __attribute__((__packed__)) {
  uint8_t flags;
} jsd_el2828_rxpdo_t;

const jsd_el2828_state_t* jsd_el2828_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el2828_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  return &self->slave_states[slave_id].el2828;
}

void jsd_el2828_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el2828_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_el2828_rxpdo_t* rxpdo =
      (jsd_el2828_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;

  int ch;
  for (ch = 0; ch < JSD_EL2828_NUM_CHANNELS; ch++) {
    uint8_t output = self->slave_states[slave_id].el2828.output[ch];

    if (output > 0) {
      rxpdo->flags |= 0x01 << ch;
    } else {
      rxpdo->flags &= ~(0x01 << ch);
    }
  }
}

void jsd_el2828_write_single_channel(jsd_t* self, uint16_t slave_id,
                                     uint8_t channel, uint8_t output) {
  assert(self);
  assert(jsd_el2828_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  self->slave_states[slave_id].el2828.output[channel] = output;
}

void jsd_el2828_write_all_channels(jsd_t* self, uint16_t slave_id,
                                   uint8_t output[JSD_EL2828_NUM_CHANNELS]) {
  int ch;
  for (ch = 0; ch < JSD_EL2828_NUM_CHANNELS; ch++) {
    jsd_el2828_write_single_channel(self, slave_id, ch, output[ch]);
  }
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el2828_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el2828_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  jsd_slave_config_t* config = &self->slave_configs[slave_id];

  // no PO2SO callback for 2828 devices, so set the success flag now
  config->PO2SO_success = true;

  return true;
}

bool jsd_el2828_product_code_is_compatible(uint32_t product_code) {
  return product_code == JSD_EL2828_PRODUCT_CODE;
}
