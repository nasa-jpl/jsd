#include "jsd/jsd_el5042.h"

#include <assert.h>
#include <string.h>

#include "jsd/jsd_sdo.h"

/****************************************************
 * Public functions
 ****************************************************/

const jsd_el5042_state_t* jsd_el5042_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el5042_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_el5042_state_t* state = &self->slave_states[slave_id].el5042;
  return state;
}

void jsd_el5042_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el5042_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_el5042_state_t* state = &self->slave_states[slave_id].el5042;

  const jsd_el5042_txpdo_t* txpdo =
      (jsd_el5042_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  for (int ch = 0; ch < JSD_EL5042_NUM_CHANNELS; ++ch) {
    state->position[ch] = txpdo->channel[ch].position;

    state->warning[ch]             = (txpdo->channel[ch].status >> 0) & 0x01;
    state->error[ch]               = (txpdo->channel[ch].status >> 1) & 0x01;
    state->ready[ch]               = (txpdo->channel[ch].status >> 2) & 0x01;
    state->diag[ch]                = (txpdo->channel[ch].status >> 4) & 0x01;
    state->txpdo_state[ch]         = (txpdo->channel[ch].status >> 5) & 0x01;
    state->input_cycle_counter[ch] = (txpdo->channel[ch].status >> 6) & 0x03;
  }
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el5042_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el5042_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_el5042_PO2SO_config;

  return true;
}

int jsd_el5042_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(jsd_el5042_product_code_is_compatible(
      ecx_context->slavelist[slave_id].eep_id));

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within the ecx_context and extract it here.
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;

  jsd_slave_config_t* config = &slave_configs[slave_id];

  for (int ch = 0; ch < JSD_EL5042_NUM_CHANNELS; ++ch) {
    // Index for settings is 0x80n8, where n is channel number (e.g. ch2 =
    // 0x8018).
    uint32_t sdo_channel_index = 0x8008 + (0x10 * ch);

    // Set the encoder supply voltage, either 50 for 5V or 90 for 9V.
    //uint8_t supply_voltage = 50; // 5V (default)
    //if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    //0x12, JSD_SDO_DATA_U8, &supply_voltage)) {
      //return 0;
    //}

    // Set the CRC inversion
    uint8_t CRC_invert = 1; // True correspond to CRC transmitted inverted
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x03, JSD_SDO_DATA_U8, &CRC_invert)) {
      return 0;
    }

    // Set the BiSS clock frequency.
    // 0 -> 10 MHz
    // 1 -> 5 MHz
    // 2 -> 3.33 MHz
    // 3 -> 2.5 MHz
    // 4 -> 2 MHz
    // 9 -> 1 MHz
    // 17 -> 500 kHz
    // 19 -> 250 kHz
    uint8_t clock_frequency = 1; // 5 MHz
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x13, JSD_SDO_DATA_U8, &clock_frequency)) {
      return 0;
    }

    // Set the number of multiturn bits
    uint8_t multiturn_bits = 0;
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x15, JSD_SDO_DATA_U8, &multiturn_bits)) {
      return 0;
    }
  }

  config->PO2SO_success = true;
  return 1;
}

bool jsd_el5042_product_code_is_compatible(uint32_t product_code) {
  return product_code == JSD_EL5042_PRODUCT_CODE;
}
