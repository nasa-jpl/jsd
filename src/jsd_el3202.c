#include "jsd/jsd_el3202.h"

#include <assert.h>
#include <string.h>

#include "jsd/jsd_sdo.h"

const char* jsd_el3202_element_strings[] = {
    [JSD_EL3202_ELEMENT_PT100]         = "PT100",
    [JSD_EL3202_ELEMENT_NI100]         = "NI100",
    [JSD_EL3202_ELEMENT_PT1000]        = "PT1000",
    [JSD_EL3202_ELEMENT_PT500]         = "PT500",
    [JSD_EL3202_ELEMENT_PT200]         = "PT200",
    [JSD_EL3202_ELEMENT_NI1000]        = "NI1000",
    [JSD_EL3202_ELEMENT_NI1000_TK1500] = "NI1000 TK1500",
    [JSD_EL3202_ELEMENT_NI120]         = "NI120",
    [JSD_EL3202_ELEMENT_OHMS4096]      = "Ohms 4096",
    [JSD_EL3202_ELEMENT_OHMS1024]      = "Ohms 1024",
    [JSD_EL3202_ELEMENT_KT100_ET_AL]   = "KT100 et al.",
};

const char* jsd_el3202_connection_strings[] = {
    [JSD_EL3202_CONNECTION_2WIRE]         = "2 Wire",
    [JSD_EL3202_CONNECTION_3WIRE]         = "3 Wire",
    [JSD_EL3202_CONNECTION_4WIRE]         = "4 Wire",
    [JSD_EL3202_CONNECTION_NOT_CONNECTED] = "Not Connected",
};

const char* jsd_el3202_presentation_strings[] = {
    [JSD_EL3202_PRESENTATION_SIGNED]   = "Signed",
    [1]                                = "unused presentation",
    [JSD_EL3202_PRESENTATION_HIGH_RES] = "High Res",
};

/****************************************************
 * Public functions
 ****************************************************/
const jsd_el3202_state_t* jsd_el3202_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3202_PRODUCT_CODE);

  return &self->slave_states[slave_id].el3202;
}

void jsd_el3202_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3202_PRODUCT_CODE);

  jsd_el3202_state_t*  state  = &self->slave_states[slave_id].el3202;
  jsd_el3202_config_t* config = &self->slave_configs[slave_id].el3202;

  jsd_el3202_txpdo_t* txpdo =
      (jsd_el3202_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  int ch;
  for (ch = 0; ch < JSD_EL3202_NUM_CHANNELS; ch++) {
    state->adc_value[ch] = txpdo->channel[ch].value;
    state->output_eu[ch] =
        jsd_el3202_output_from_config(txpdo->channel[ch].value, config, ch);

    state->underrange[ch]   = (txpdo->channel[ch].flags >> 0) & 0x01;
    state->overrange[ch]    = (txpdo->channel[ch].flags >> 1) & 0x01;
    state->limit1[ch]       = (txpdo->channel[ch].flags >> 2) & 0x03;
    state->limit2[ch]       = (txpdo->channel[ch].flags >> 4) & 0x03;
    state->error[ch]        = (txpdo->channel[ch].flags >> 6) & 0x01;
    state->txPDO_state[ch]  = (txpdo->channel[ch].flags >> 14) & 0x01;
    state->txPDO_toggle[ch] = (txpdo->channel[ch].flags >> 15) & 0x01;
  }
}

void jsd_el3202_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3202_PRODUCT_CODE);

  jsd_async_sdo_process_response(self, slave_id);
}
/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el3202_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3202_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_el3202_PO2SO_config;

  return true;
}

int jsd_el3202_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_EL3202_PRODUCT_CODE);

  // cast the void* to slave_config
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
  for (ch = 0; ch < JSD_EL3202_NUM_CHANNELS; ch++) {
    MSG("\t Ch%d Config:", ch);
    MSG("\t\tElement: %s",
        jsd_el3202_element_strings[config->el3202.element[ch]]);
    MSG("\t\tFilter: %s",
        jsd_beckhoff_filter_strings[config->el3202.filter[ch]]);
    MSG("\t\tConnection: %s",
        jsd_el3202_connection_strings[config->el3202.connection[ch]]);
    MSG("\t\tWire Resistance: %f", config->el3202.wire_resistance[ch]);
    MSG("\t\tPresentation: %s",
        jsd_el3202_presentation_strings[config->el3202.presentation[ch]]);

    // register is 0x80n0, where n is channel number (e.g. ch4 = 0x8040)
    uint32_t sdo_channel_index = 0x8000 + (0x10 * ch);

    uint16_t element = config->el3202.element[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x19, JSD_SDO_DATA_U16, &element)) {
      return 0;
    }

    uint16_t filter = config->el3202.filter[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x15, JSD_SDO_DATA_U16, &filter)) {
      return 0;
    }

    uint16_t connection = config->el3202.connection[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x1A, JSD_SDO_DATA_U16, &connection)) {
      return 0;
    }

    int16_t wire_resistance = config->el3202.wire_resistance[ch] * 32;
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x1B, JSD_SDO_DATA_I16, &wire_resistance)) {
      return 0;
    }

    uint8_t presentation = config->el3202.presentation[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x02, JSD_SDO_DATA_U8, &presentation)) {
      return 0;
    }
  }

  config->PO2SO_success = true;
  return 1;
}

double jsd_el3202_output_from_config(int16_t              adc_value,
                                     jsd_el3202_config_t* config,
                                     uint16_t             channel) {
  assert(config->presentation[channel] < JSD_EL3202_NUM_PRESENTATIONS);
  assert(config->element[channel] < JSD_EL3202_NUM_ELEMENTS);

  double temp_scale_factor = 1;

  switch (config->presentation[channel]) {
    case JSD_EL3202_PRESENTATION_SIGNED:
      temp_scale_factor = 0.1;
      break;
    case JSD_EL3202_PRESENTATION_HIGH_RES:
      temp_scale_factor = 0.01;
      break;
    default:
      WARNING("Bad/Unsupported presentation setting");
      break;
  }

  uint16_t* unsigned_adc_value = (uint16_t*)&adc_value;
  switch (config->element[channel]) {
    case JSD_EL3202_ELEMENT_OHMS4096:
      return *unsigned_adc_value / 16.0;
      break;

    case JSD_EL3202_ELEMENT_OHMS1024:
      return *unsigned_adc_value / 64.0;
      break;

    default:  // is a standard RTD element
      return adc_value * temp_scale_factor;
      break;
  }
}
