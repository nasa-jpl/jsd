#include "jsd/jsd_el3314.h"

#include <assert.h>
#include <string.h>

#include "jsd/jsd_sdo.h"

const char* jsd_el3314_element_strings[] = {
    [JSD_EL3314_ELEMENT_TYPE_K] = "TYPE_K",
    [JSD_EL3314_ELEMENT_TYPE_J] = "TYPE_J",
    [JSD_EL3314_ELEMENT_TYPE_L] = "TYPE_L",
    [JSD_EL3314_ELEMENT_TYPE_E] = "TYPE_E",
    [JSD_EL3314_ELEMENT_TYPE_T] = "TYPE_T",
    [JSD_EL3314_ELEMENT_TYPE_N] = "TYPE_N",
    [JSD_EL3314_ELEMENT_TYPE_U] = "TYPE_U",
    [JSD_EL3314_ELEMENT_TYPE_B] = "TYPE_B",
    [JSD_EL3314_ELEMENT_TYPE_R] = "TYPE_R",
    [JSD_EL3314_ELEMENT_TYPE_S] = "TYPE_S",
    [JSD_EL3314_ELEMENT_TYPE_C] = "TYPE_C",
};

const char* jsd_el3314_presentation_strings[] = {
    [JSD_EL3314_PRESENTATION_SIGNED]   = "Signed",
    [1]                                = "unused presentation",
    [JSD_EL3314_PRESENTATION_HIGH_RES] = "High Res",
};

/****************************************************
 * Public functions
 ****************************************************/
const jsd_el3314_state_t* jsd_el3314_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el3314_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  return &self->slave_states[slave_id].el3314;
}

void jsd_el3314_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el3314_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_el3314_state_t*  state  = &self->slave_states[slave_id].el3314;
  jsd_el3314_config_t* config = &self->slave_configs[slave_id].el3314;

  jsd_el3314_txpdo_t* txpdo =
      (jsd_el3314_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  int ch;
  for (ch = 0; ch < JSD_EL3314_NUM_CHANNELS; ch++) {
    state->adc_value[ch] = txpdo->channel[ch].value;
    state->output_eu[ch] =
        jsd_el3314_output_from_config(txpdo->channel[ch].value, config, ch);

    state->underrange[ch]   = (txpdo->channel[ch].flags >> 0) & 0x01;
    state->overrange[ch]    = (txpdo->channel[ch].flags >> 1) & 0x01;
    state->error[ch]        = (txpdo->channel[ch].flags >> 6) & 0x01;
    state->txPDO_state[ch]  = (txpdo->channel[ch].flags >> 14) & 0x01;
    state->txPDO_toggle[ch] = (txpdo->channel[ch].flags >> 15) & 0x01;
  }
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el3314_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el3314_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_el3314_PO2SO_config;

  return true;
}

int jsd_el3314_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(jsd_el3314_product_code_is_compatible(
      ecx_context->slavelist[slave_id].eep_id));

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
  for (ch = 0; ch < JSD_EL3314_NUM_CHANNELS; ch++) {
    MSG("\t Ch%d Config:", ch);
    MSG("\t\tElement: %s",
        jsd_el3314_element_strings[config->el3314.element[ch]]);
    MSG("\t\tFilter: %s",
        jsd_beckhoff_filter_strings[config->el3314.filter[ch]]);
    MSG("\t\tPresentation: %s",
        jsd_el3314_presentation_strings[config->el3314.presentation[ch]]);

    // register is 0x80n0, where n is channel number (e.g. ch4 = 0x8040)
    uint32_t sdo_channel_index = 0x8000 + (0x10 * ch);

    uint16_t element = config->el3314.element[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x19, JSD_SDO_DATA_U16, &element)) {
      return 0;
    }

    uint16_t filter = config->el3314.filter[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x15, JSD_SDO_DATA_U16, &filter)) {
      return 0;
    }

    uint8_t presentation = config->el3314.presentation[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x02, JSD_SDO_DATA_U8, &presentation)) {
      return 0;
    }
  }

  config->PO2SO_success = true;
  return 1;
}

double jsd_el3314_output_from_config(int16_t              adc_value,
                                     jsd_el3314_config_t* config,
                                     uint16_t             channel) {
  assert(config->presentation[channel] < JSD_EL3314_NUM_PRESENTATIONS);
  assert(config->element[channel] < JSD_EL3314_NUM_ELEMENTS);

  double temp_scale_factor = 1;

  switch (config->presentation[channel]) {
    case JSD_EL3314_PRESENTATION_SIGNED:
      temp_scale_factor = 0.1;
      break;
    case JSD_EL3314_PRESENTATION_HIGH_RES:
      temp_scale_factor = 0.01;
      break;
    default:
      WARNING("Bad/Unsupported presentation setting");
      break;
  }

  return (double)adc_value * temp_scale_factor;
}

bool jsd_el3314_product_code_is_compatible(uint32_t product_code) {
  return product_code == JSD_EL3314_PRODUCT_CODE;
}
