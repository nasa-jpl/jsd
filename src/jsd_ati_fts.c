#include "jsd/jsd_ati_fts.h"

#include <assert.h>
#include <string.h>

#include "jsd/jsd_sdo.h"

/****************************************************
 * Public functions
 ****************************************************/

const jsd_ati_fts_state_t* jsd_ati_fts_get_state(jsd_t*   self,
                                                 uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_ATI_FTS_PRODUCT_CODE);

  jsd_ati_fts_state_t* state = &self->slave_states[slave_id].ati_fts;
  return state;
}

void jsd_ati_fts_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_ATI_FTS_PRODUCT_CODE);

  jsd_slave_config_t*  config = &self->slave_configs[slave_id];
  jsd_ati_fts_state_t* state  = &self->slave_states[slave_id].ati_fts;

  jsd_ati_fts_txpdo_t* txpdo =
      (jsd_ati_fts_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  state->fx =
      (double)txpdo->fx_counts / (double)config->ati_fts.counts_per_force;
  state->fy =
      (double)txpdo->fy_counts / (double)config->ati_fts.counts_per_force;
  state->fz =
      (double)txpdo->fz_counts / (double)config->ati_fts.counts_per_force;

  state->tx =
      (double)txpdo->tx_counts / (double)config->ati_fts.counts_per_torque;
  state->ty =
      (double)txpdo->ty_counts / (double)config->ati_fts.counts_per_torque;
  state->tz =
      (double)txpdo->tz_counts / (double)config->ati_fts.counts_per_torque;

  // This logic prevente excessive error messages and likely saves some
  // computation
  if (txpdo->status_code != state->status_code) {
    state->active_error =
        jsd_ati_fts_parse_status_code(txpdo->status_code, slave_id);
    state->status_code = txpdo->status_code;
  }

  state->sample_counter = txpdo->sample_counter;
}

void jsd_ati_fts_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_ATI_FTS_PRODUCT_CODE);

  jsd_slave_config_t*  config = &self->slave_configs[slave_id];
  jsd_ati_fts_rxpdo_t* rxpdo =
      (jsd_ati_fts_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;

  rxpdo->control1 = JSD_ATI_FTS_DEFAULT_WORD_CONTROL1;
  rxpdo->control1 |= (config->ati_fts.calibration << 8);

  rxpdo->control2 = JSD_ATI_FTS_DEFAULT_WORD_CONTROL2;

  jsd_async_sdo_process_response(self, slave_id);
}
/****************************************************
 * Private functions
 ****************************************************/

bool jsd_ati_fts_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_ATI_FTS_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man == JSD_ATI_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_ati_fts_PO2SO_config;

  return true;
}

int jsd_ati_fts_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_ATI_FTS_PRODUCT_CODE);

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within th ecx_context and extract it here
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;

  jsd_slave_config_t* config = &slave_configs[slave_id];

  MSG("Configuring slave no: %u,  SII inferred name: %s", slave_id,
      ecx_context->slavelist[slave_id].name);
  MSG("\t Configured name: %s", config->name);

  // Check Calibration value is valid
  if (config->ati_fts.calibration > JSD_ATI_FTS_MAX_CALIBRATION_VALUE) {
    ERROR("ATI-FTS: Calibration (%u) exceeds max(%u), check this input",
          config->ati_fts.calibration, JSD_ATI_FTS_MAX_CALIBRATION_VALUE);
    return 0;
  }

  // Set Calibration
  uint32_t control1 = JSD_ATI_FTS_DEFAULT_WORD_CONTROL1;
  control1 |= (config->ati_fts.calibration << 8);

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x7010, 0x01,
                                  JSD_SDO_DATA_U32, &control1)) {
    return 0;
  }

  // Read firmware version and print it out
  uint16_t major, minor, revision;
  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id, 0x2090, 0x01,
                                  JSD_SDO_DATA_U16, &major)) {
    return 0;
  }

  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id, 0x2090, 0x02,
                                  JSD_SDO_DATA_U16, &minor)) {
    return 0;
  }

  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id, 0x2090, 0x03,
                                  JSD_SDO_DATA_U16, &revision)) {
    return 0;
  }
  MSG("\t ATI Firmware version: %d.%d.%d", major, minor, revision);

  jsd_ati_fts_calibration_do_t cal_do;
  int cal_do_size = (int)sizeof(jsd_ati_fts_calibration_do_t);
  if (!jsd_sdo_get_ca_param_blocking(ecx_context, slave_id, 0x2040, 0x01,
                                     &cal_do_size, &cal_do)) {
    return 0;
  }

  const char* force_unit_str =
      jsd_ati_fts_force_unit_to_string(cal_do.force_units);
  const char* torque_unit_str =
      jsd_ati_fts_torque_unit_to_string(cal_do.torque_units);

  MSG("\t ATI Serial Number: %s", cal_do.serial_number);
  MSG("\t ATI Calibration Integer (%u) maps to: %s",
      config->ati_fts.calibration, cal_do.calibration_part_number);
  MSG("\t ATI Calibration Family: %s", cal_do.calibration_family);
  MSG("\t ATI Calibration Date: %s", cal_do.calibration_date);
  MSG("\t ATI force units: %s (%u)", force_unit_str, cal_do.force_units);
  MSG("\t ATI torque units: %s (%u)", torque_unit_str, cal_do.torque_units);
  MSG("\t ATI counts_per_force: %u", cal_do.counts_per_force);
  MSG("\t ATI counts_per_torque: %u", cal_do.counts_per_torque);

  config->ati_fts.counts_per_force  = cal_do.counts_per_force;
  config->ati_fts.counts_per_torque = cal_do.counts_per_torque;

  // Read the status code and check for calibration fault
  uint32_t status_code;
  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id, 0x6010, 0x00,
                                  JSD_SDO_DATA_U32, &status_code)) {
    return 0;
  }

  if (jsd_ati_fts_parse_status_code(status_code, slave_id)) {
    ERROR("Failed to initialize ATI FTS without fault");
    return 0;
  }

  config->PO2SO_success = true;
  return 1;
}

bool jsd_ati_fts_parse_status_code(uint32_t status_code, uint16_t slave_id) {
  bool active_fault = false;

  if ((status_code >> 0) % 0x01) {
    MSG("ATI FTS [%u] - Monitor Condition Tripped", slave_id);
  }

  if ((status_code >> 1) & 0x01) {
    MSG("ATI FTS [%u] - Supply Out of Range", slave_id);
  }

  if ((status_code >> 3) & 0x01) {
    MSG("ATI FTS [%u] - Vbridge Volts Out of Range", slave_id);
  }

  if ((status_code >> 4) & 0x01) {
    MSG("ATI FTS [%u] - Vbridge Current Out of Range", slave_id);
  }

  if ((status_code >> 5) & 0x01) {
    MSG("ATI FTS [%u] - DPOT Fault", slave_id);
  }

  if ((status_code >> 6) & 0x01) {
    MSG("ATI FTS [%u] - EEPROM Fault", slave_id);
  }

  if ((status_code >> 7) & 0x01) {
    MSG("ATI FTS [%u] - DAC Fault", slave_id);
  }

  if ((status_code >> 28) & 0x01) {
    MSG("ATI FTS [%u] - Simulated Error", slave_id);
  }

  if ((status_code >> 29) & 0x01) {
    MSG("ATI FTS [%u] - Calibration Checksum Out of Range", slave_id);
  }

  if ((status_code >> 30) & 0x01) {
    MSG("ATI FTS [%u] - Saturation", slave_id);
  }

  if ((status_code >> 31) & 0x01) {
    MSG("ATI FTS [%u] - Active Fault", slave_id);
    active_fault = true;
  }

  return active_fault;
}

const char* jsd_ati_fts_force_unit_to_string(uint8_t force_units) {
  switch (force_units) {
    case 1:
      return "Lbf";
      break;
    case 2:
      return "N";
      break;
    case 3:
      return "Klbf";
      break;
    case 4:
      return "KN";
      break;
    case 5:
      return "Kg";
      break;
    default:
      return "Unknown Force Unit";
      break;
  }
}

const char* jsd_ati_fts_torque_unit_to_string(uint8_t torque_units) {
  switch (torque_units) {
    case 1:
      return "Lbf-in";
      break;
    case 2:
      return "Lbf-ft";
      break;
    case 3:
      return "N-m";
      break;
    case 4:
      return "N-mm";
      break;
    case 5:
      return "Kg-cm";
      break;
    case 6:
      return "Kg-m";
      break;
    default:
      return "Unknown Torque Unit";
      break;
  }
}
