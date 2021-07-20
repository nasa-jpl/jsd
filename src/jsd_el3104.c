#include "jsd/jsd_el3104.h"

#include <assert.h>
#include <string.h>

#include "jsd/jsd_sdo.h"

/****************************************************
 * Public functions
 ****************************************************/

const jsd_el3104_state_t* jsd_el3104_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3104_PRODUCT_CODE);

  jsd_el3104_state_t* state = &self->slave_states[slave_id].el3104;
  return state;
}

void jsd_el3104_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3104_PRODUCT_CODE);

  jsd_el3104_state_t* state = &self->slave_states[slave_id].el3104;

  jsd_el3104_txpdo_t* txpdo =
      (jsd_el3104_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  int ch;
  for (ch = 0; ch < JSD_EL3104_NUM_CHANNELS; ch++) {
    state->adc_value[ch] = txpdo->channel[ch].value;

    state->voltage[ch] = (double)state->adc_value[ch] / 3276.8;

    state->underrange[ch]   = (txpdo->channel[ch].flags >> 0) & 0x01;
    state->overrange[ch]    = (txpdo->channel[ch].flags >> 1) & 0x01;
    state->error[ch]        = (txpdo->channel[ch].flags >> 6) & 0x01;
    state->sync_error[ch]   = (txpdo->channel[ch].flags >> 13) & 0x01;
    state->txPDO_state[ch]  = (txpdo->channel[ch].flags >> 14) & 0x01;
    state->txPDO_toggle[ch] = (txpdo->channel[ch].flags >> 15) & 0x01;
  }
}

void jsd_el3104_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3104_PRODUCT_CODE);

  jsd_async_sdo_process_response(self, slave_id);
}
/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el3104_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3104_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_el3104_PO2SO_config;

  return true;
}

int jsd_el3104_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_EL3104_PRODUCT_CODE);

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within th ecx_context and extract it here
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;

  jsd_slave_config_t* config = &slave_configs[slave_id];

  // Reset to factory default
  uint32_t reset_word = JSD_BECKHOFF_RESET_WORD;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, JSD_BECKHOFF_RESET_SDO,
                                  JSD_BECKHOFF_RESET_SUBIND, JSD_SDO_DATA_U32,
                                  &reset_word)) {
    return 0;
  }

  MSG("Configuring slave no: %u,  SII inferred name: %s", slave_id,
      ecx_context->slavelist[slave_id].name);
  MSG("\t Configured name: %s", config->name);

  int ch;
  for (ch = 0; ch < JSD_EL3104_NUM_CHANNELS; ch++) {
    // register is 0x80n0, where n is channel number (e.g. ch4 = 0x8040)
    uint32_t sdo_channel_index = 0x8000 + (0x10 * ch);

    // Enable Filter (not explicitly required, always enabled)
    uint8_t enable_filter = 1;
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x06, JSD_SDO_DATA_U8, &enable_filter)) {
      return 0;
    }

    // Set Filter Option
    uint16_t filter_opt = 2; // 400 hz IIR Filter, fastest rate. Ref:
                             // el31xxen.pdf, page 203 Filter settings
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x15, JSD_SDO_DATA_U16, &filter_opt)) {
      return 0;
    }
  }

  config->PO2SO_success = true;
  return 1;
}
