#include "jsd/jsd_el3004.h"

#include <assert.h>
#include <string.h>

#include "jsd/jsd_sdo.h"

/****************************************************
 * Public functions
 ****************************************************/

const jsd_el3004_state_t* jsd_el3004_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3004_PRODUCT_CODE);

  jsd_el3004_state_t* state = &self->slave_states[slave_id].el3004;
  return state;
}

void jsd_el3004_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3004_PRODUCT_CODE);

  jsd_el3004_state_t* state = &self->slave_states[slave_id].el3004;

  const jsd_el3004_txpdo_t* txpdo =
      (jsd_el3004_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  for (int ch = 0; ch < JSD_EL3004_NUM_CHANNELS; ++ch) {
    state->adc_value[ch] = txpdo->channel[ch].value;

    // EL3004 has a +/-10V range. Assuming it's configured to use 2's compliment, adc value will
    // be a signed value between 32767 and -32768
    if (state->adc_value[ch] > 0) {
      state->adc_value[ch] = (double)state->adc_value[ch]/32767.0 * 10.0;
    } else {
      state->adc_value[ch] = (double)state->adc_value[ch]/32768.0 * 10.0;
    }

    // EL3004 status data is 1-byte long.
    state->underrange[ch] = (txpdo->channel[ch].flags >> 0) & 0x01;
    state->overrange[ch]  = (txpdo->channel[ch].flags >> 1) & 0x01;
    state->error[ch]      = (txpdo->channel[ch].flags >> 6) & 0x01;
  }
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el3004_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3004_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_el3004_PO2SO_config;

  return true;
}

int jsd_el3004_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_EL3004_PRODUCT_CODE);

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within the ecx_context and extract it here.
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;

  jsd_slave_config_t* config = &slave_configs[slave_id];

  // Reset to factory default.
  uint32_t reset_word = JSD_BECKHOFF_RESET_WORD;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, JSD_BECKHOFF_RESET_SDO,
                                  JSD_BECKHOFF_RESET_SUBIND, JSD_SDO_DATA_U32,
                                  &reset_word)) {
    return 0;
  }

  MSG("Configuring slave no: %u, SII inferred name: %s", slave_id,
      ecx_context->slavelist[slave_id].name);
  MSG("\t Configured name: %s", config->name);

  for (int ch = 0; ch < JSD_EL3004_NUM_CHANNELS; ++ch) {
    // Index for settings is 0x80n0, where n is channel number (e.g. ch2 =
    // 0x8010).
    /*
    uint32_t sdo_channel_index = 0x8000 + (0x10 * ch);

    // Don't need to set presentation as 2's compliment is the default,
    // and we factor reset.
    
    // Enable digital filter on read inputs (synchronized with timer inside
    // terminal).
    uint8_t enable_filter = 1;
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x06, JSD_SDO_DATA_U8, &enable_filter)) {
      return 0;
    }

    // Set filter option.
    uint16_t filter_opt =
        2;  // 1 kHz IIR filter, fastest rate. Refer to el30xen.pdf, page 269.
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x15, JSD_SDO_DATA_U16, &filter_opt)) {
      return 0;
    }
    */
  }

  config->PO2SO_success = true;
  return 1;
}
