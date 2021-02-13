#include "jsd/jsd_el3602.h"

#include <assert.h>
#include <string.h>

#include "jsd/jsd_sdo.h"

const char* jsd_el3602_range_strings[] = {
    [JSD_EL3602_RANGE_10V]   = "10V Range",
    [JSD_EL3602_RANGE_5V]    = "5V Range",
    [JSD_EL3602_RANGE_2_5V]  = "2.5V Range",
    [JSD_EL3602_RANGE_1_25V] = "1.25V Range",
    [JSD_EL3602_RANGE_75MV]  = "75mV Range",
    [JSD_EL3602_RANGE_200MV] = "200mV Range",
};

/****************************************************
 * Public functions
 ****************************************************/

const jsd_el3602_state_t* jsd_el3602_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3602_PRODUCT_CODE);

  jsd_el3602_state_t* state = &self->slave_states[slave_id].el3602;
  return state;
}

void jsd_el3602_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3602_PRODUCT_CODE);

  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  jsd_el3602_state_t* state  = &self->slave_states[slave_id].el3602;

  jsd_el3602_txpdo_t* txpdo =
      (jsd_el3602_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  int ch;
  for (ch = 0; ch < JSD_EL3602_NUM_CHANNELS; ch++) {
    state->adc_value[ch] = txpdo->channel[ch].value;

    state->voltage[ch] = (double)state->adc_value[ch] *
                         jsd_el3602_range_factor[config->el3602.range[ch]] /
                         JSD_EL3602_DAQ_RESOLUTION;

    state->underrange[ch]   = (txpdo->channel[ch].flags >> 0) & 0x01;
    state->overrange[ch]    = (txpdo->channel[ch].flags >> 1) & 0x01;
    state->limit1[ch]       = (txpdo->channel[ch].flags >> 2) & 0x03;
    state->limit2[ch]       = (txpdo->channel[ch].flags >> 4) & 0x03;
    state->error[ch]        = (txpdo->channel[ch].flags >> 6) & 0x01;
    state->txPDO_state[ch]  = (txpdo->channel[ch].flags >> 14) & 0x01;
    state->txPDO_toggle[ch] = (txpdo->channel[ch].flags >> 15) & 0x01;
  }
}

void jsd_el3602_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3602_PRODUCT_CODE);

  jsd_async_sdo_process_response(self, slave_id);
}
/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el3602_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3602_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_el3602_PO2SO_config;

  return true;
}

int jsd_el3602_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_EL3602_PRODUCT_CODE);

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
  for (ch = 0; ch < JSD_EL3602_NUM_CHANNELS; ch++) {
    MSG("\t range[%d]: %s", ch,
        jsd_el3602_range_strings[config->el3602.range[ch]]);
    MSG("\t Ch%d Filter: %s", ch,
        jsd_beckhoff_filter_strings[config->el3602.filter[ch]]);

    // register is 0x80n0, where n is channel number (e.g. ch4 = 0x8040)
    uint32_t sdo_channel_index = 0x8000 + (0x10 * ch);

    // Set Range
    assert(config->el3602.range[ch] < JSD_EL3602_NUM_RANGES);
    uint16_t range = config->el3602.range[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x19, JSD_SDO_DATA_U16, &range)) {
      return 0;
    }

    // Enable Filter (not explicitly required, always enabled)
    uint8_t enable_filter = 1;
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x06, JSD_SDO_DATA_U8, &enable_filter)) {
      return 0;
    }

    // Set Filter Option
    uint16_t filter_opt = config->el3602.filter[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x15, JSD_SDO_DATA_U16, &filter_opt)) {
      return 0;
    }

    // Set limit1
    if (config->el3602.limit1_enable[ch]) {
      int32_t limit_value = config->el3602.limit1_voltage[ch] /
                            jsd_el3602_range_factor[config->el3602.range[ch]] *
                            JSD_EL3602_DAQ_RESOLUTION / 2.0;
      if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                      0x13, JSD_SDO_DATA_I32, &limit_value)) {
        return 0;
      }
      MSG("Enabling limit1 on channel %d to value of %d", ch, limit_value);

      uint8_t enable_limit = 1;
      if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                      0x7, JSD_SDO_DATA_U8, &enable_limit)) {
        return 0;
      }
    }

    // Set limit2
    if (config->el3602.limit2_enable[ch]) {
      int32_t limit_value = config->el3602.limit2_voltage[ch] /
                            jsd_el3602_range_factor[config->el3602.range[ch]] *
                            JSD_EL3602_DAQ_RESOLUTION / 2.0;
      if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                      0x14, JSD_SDO_DATA_I32, &limit_value)) {
        return 0;
      }
      MSG("Enabling limit2 on channel %d to value of %d", ch, limit_value);

      uint8_t enable_limit = 1;
      if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                      0x8, JSD_SDO_DATA_U8, &enable_limit)) {
        return 0;
      }
    }
  }

  config->PO2SO_success = true;
  return 1;
}
