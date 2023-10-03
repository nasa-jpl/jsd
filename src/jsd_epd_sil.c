#include "jsd/jsd_epd_sil.h"

#include <assert.h>
#include <string.h>

#include "ethercat.h"
#include "jsd/jsd.h"
#include "jsd/jsd_elmo_common.h"
#include "jsd/jsd_epd_common.h"
#include "jsd/jsd_sdo.h"

#define JSD_EPD_SIL_MAX_ERROR_POPS_PER_CYCLE (5)

#define JSD_EPD_SIL_MAX_OBJS_PER_PDO_MAPPING_PARAMETER (8)

// Subindex offsets for SIL R1/R2 outputs.
#define JSD_EPD_SIL_R1_OUTPUTS_SUBINDEX_OFFSET (129)
#define JSD_EPD_SIL_R2_OUTPUTS_SUBINDEX_OFFSET (65)

// WARNING: Only use this macro on raw arrays. Do not use it on array function
// parameters or pointers.
#define JSD_EPD_SIL_ARRAY_SIZE(array) (sizeof((array)) / sizeof((array)[0]))

static int jsd_epd_sil_min(int lhs, int rhs) { return (lhs < rhs) ? lhs : rhs; }

/****************************************************
 * Public functions
 ****************************************************/

uint16_t jsd_epd_sil_lc_to_do(char letter_command[2]) {
  return jsd_epd_lc_to_do_impl(letter_command);
}

const jsd_epd_sil_state_t* jsd_epd_sil_get_state(jsd_t*   self,
                                                 uint16_t slave_id) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  return &self->slave_states[slave_id].epd_sil.pub;
}

void jsd_epd_sil_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_slave_config_t* config = &self->slave_configs[slave_id];

  // Copy TxPDO data from SOEM's IOmap
  assert((sizeof(jsd_epd_sil_txpdo_data_t) +
          (config->epd_sil.sil_r1_outputs_num * 4) +
          (config->epd_sil.sil_r2_outputs_num * 8)) ==
         self->ecx_context.slavelist[slave_id].Ibytes);
  memcpy(&self->slave_states[slave_id].epd_sil.default_txpdo,
         self->ecx_context.slavelist[slave_id].inputs,
         sizeof(self->slave_states[slave_id].epd_sil.default_txpdo));
  memcpy(&self->slave_states[slave_id].epd_sil.sil_r1_outputs,
         self->ecx_context.slavelist[slave_id].inputs +
             sizeof(self->slave_states[slave_id].epd_sil.default_txpdo),
         config->epd_sil.sil_r1_outputs_num * 4);
  memcpy(&self->slave_states[slave_id].epd_sil.sil_r2_outputs,
         self->ecx_context.slavelist[slave_id].inputs +
             (sizeof(self->slave_states[slave_id].epd_sil.default_txpdo) +
              (config->epd_sil.sil_r1_outputs_num * 4)),
         config->epd_sil.sil_r2_outputs_num * 8);

  jsd_epd_sil_update_state_from_PDO_data(self, slave_id);
}

void jsd_epd_sil_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_slave_config_t* config = &self->slave_configs[slave_id];

  jsd_epd_sil_process_state_machine(self, slave_id);

  // Copy RxPDO data into SOEM's IOmap
  assert((sizeof(jsd_epd_sil_rxpdo_data_t) +
          (config->epd_sil.sil_r1_inputs_num * 4) +
          (config->epd_sil.sil_r2_inputs_num * 8)) ==
         self->ecx_context.slavelist[slave_id].Obytes);
  memcpy(self->ecx_context.slavelist[slave_id].outputs,
         &self->slave_states[slave_id].epd_sil.default_rxpdo,
         sizeof(self->slave_states[slave_id].epd_sil.default_rxpdo));
  memcpy(self->ecx_context.slavelist[slave_id].outputs +
             sizeof(self->slave_states[slave_id].epd_sil.default_rxpdo),
         &self->slave_states[slave_id].epd_sil.sil_r1_inputs,
         config->epd_sil.sil_r1_inputs_num * 4);
  memcpy(self->ecx_context.slavelist[slave_id].outputs +
             (sizeof(self->slave_states[slave_id].epd_sil.default_rxpdo) +
              (config->epd_sil.sil_r1_inputs_num * 4)),
         &self->slave_states[slave_id].epd_sil.sil_r2_inputs,
         config->epd_sil.sil_r2_inputs_num * 8);
}

void jsd_epd_sil_reset(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  double now = jsd_time_get_mono_time_sec();

  if ((now - self->slave_states[slave_id].epd_sil.last_reset_time) >
      JSD_EPD_RESET_DERATE_SEC) {
    self->slave_states[slave_id].epd_sil.new_reset       = true;
    self->slave_states[slave_id].epd_sil.last_reset_time = now;

    // TODO(dloret): EGD code clears fault_code/emcy_error_code here. However,
    // the clearing is also done when entering the OPERATION_ENABLED state. It
    // seems only one is needed and clearing in OPERATION_ENABLED is more in
    // line with only 2 states visible for user: error and non-error (i.e.
    // OPERATION_ENABLED).
  } else {
    // TODO(dloret): Remove printing to not affect real-time guarantees.
    WARNING(
        "EPD SIL Reset Derate Protection feature is preventing reset, ignoring "
        "request");
  }
}

void jsd_epd_sil_clear_errors(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  self->slave_states[slave_id].epd_sil.pub.fault_code      = JSD_EPD_FAULT_OKAY;
  self->slave_states[slave_id].epd_sil.pub.emcy_error_code = 0;
}

void jsd_epd_sil_halt(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  self->slave_states[slave_id].epd_sil.new_halt_command = true;
}

void jsd_epd_sil_set_peak_current(jsd_t* self, uint16_t slave_id,
                                  double peak_current) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_epd_sil_private_state_t* state = &self->slave_states[slave_id].epd_sil;

  state->default_rxpdo.max_current =
      peak_current * 1e6 / state->motor_rated_current;
}

void jsd_epd_sil_set_sil_r1_input(jsd_t* self, uint16_t slave_id,
                                  uint16_t subindex, int32_t value) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  assert(subindex >= 1 && subindex <= config->epd_sil.sil_r1_inputs_num);

  self->slave_states[slave_id].epd_sil.sil_r1_inputs[subindex - 1] = value;
}

void jsd_epd_sil_set_sil_r2_input(jsd_t* self, uint16_t slave_id,
                                  uint16_t subindex, double value) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  assert(subindex >= 1 && subindex <= config->epd_sil.sil_r2_inputs_num);

  self->slave_states[slave_id].epd_sil.sil_r2_inputs[subindex - 1] = value;
}

int32_t jsd_epd_sil_get_sil_r1_input(jsd_t* self, uint16_t slave_id,
                                     uint16_t subindex) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  assert(subindex >= 1 && subindex <= config->epd_sil.sil_r1_inputs_num);

  return self->slave_states[slave_id].epd_sil.sil_r1_inputs[subindex - 1];
}

double jsd_epd_sil_get_sil_r2_input(jsd_t* self, uint16_t slave_id,
                                    uint16_t subindex) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  assert(subindex >= 1 && subindex <= config->epd_sil.sil_r2_inputs_num);

  return self->slave_states[slave_id].epd_sil.sil_r2_inputs[subindex - 1];
}

int32_t jsd_epd_sil_get_sil_r1_output(jsd_t* self, uint16_t slave_id,
                                      uint16_t subindex) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  assert(subindex >= JSD_EPD_SIL_R1_OUTPUTS_SUBINDEX_OFFSET &&
         subindex <= ((JSD_EPD_SIL_R1_OUTPUTS_SUBINDEX_OFFSET - 1) +
                      config->epd_sil.sil_r1_outputs_num));

  return self->slave_states[slave_id]
      .epd_sil
      .sil_r1_outputs[subindex - JSD_EPD_SIL_R1_OUTPUTS_SUBINDEX_OFFSET];
}

double jsd_epd_sil_get_sil_r2_output(jsd_t* self, uint16_t slave_id,
                                     uint16_t subindex) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  assert(subindex >= JSD_EPD_SIL_R2_OUTPUTS_SUBINDEX_OFFSET &&
         subindex <= ((JSD_EPD_SIL_R2_OUTPUTS_SUBINDEX_OFFSET - 1) +
                      config->epd_sil.sil_r2_outputs_num));

  return self->slave_states[slave_id]
      .epd_sil
      .sil_r2_outputs[subindex - JSD_EPD_SIL_R2_OUTPUTS_SUBINDEX_OFFSET];
}

void jsd_epd_sil_async_sdo_set_sil_r1(jsd_t* self, uint16_t slave_id,
                                      uint16_t subindex, int32_t value,
                                      uint16_t app_id) {
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  if ((subindex >= 1 && subindex <= config->epd_sil.sil_r1_inputs_num) ||
      (subindex >= JSD_EPD_SIL_R1_OUTPUTS_SUBINDEX_OFFSET &&
       subindex <= ((JSD_EPD_SIL_R1_OUTPUTS_SUBINDEX_OFFSET - 1) +
                    config->epd_sil.sil_r1_outputs_num))) {
    ERROR("R1[%u] is already mapped to a PDO.", subindex);
    return;
  }
  jsd_sdo_set_param_async(self, slave_id, 0x22F3, subindex, JSD_SDO_DATA_I32,
                          &value, app_id);
}

void jsd_epd_sil_async_sdo_set_sil_r2(jsd_t* self, uint16_t slave_id,
                                      uint16_t subindex, double value,
                                      uint16_t app_id) {
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  if ((subindex >= 1 && subindex <= config->epd_sil.sil_r2_inputs_num) ||
      (subindex >= JSD_EPD_SIL_R2_OUTPUTS_SUBINDEX_OFFSET &&
       subindex <= ((JSD_EPD_SIL_R2_OUTPUTS_SUBINDEX_OFFSET - 1) +
                    config->epd_sil.sil_r2_outputs_num))) {
    ERROR("R2[%u] is already mapped to a PDO.", subindex);
    return;
  }
  jsd_sdo_set_param_async(self, slave_id, 0x22F4, subindex, JSD_SDO_DATA_DOUBLE,
                          &value, app_id);
}

void jsd_epd_sil_async_sdo_get_sil_r1(jsd_t* self, uint16_t slave_id,
                                      uint16_t subindex, uint16_t app_id) {
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  if ((subindex >= 1 && subindex <= config->epd_sil.sil_r1_inputs_num) ||
      (subindex >= JSD_EPD_SIL_R1_OUTPUTS_SUBINDEX_OFFSET &&
       subindex <= ((JSD_EPD_SIL_R1_OUTPUTS_SUBINDEX_OFFSET - 1) +
                    config->epd_sil.sil_r1_outputs_num))) {
    ERROR("R1[%u] is already mapped to a PDO.", subindex);
    return;
  }
  jsd_sdo_get_param_async(self, slave_id, 0x22F3, subindex, JSD_SDO_DATA_I32,
                          app_id);
}

void jsd_epd_sil_async_sdo_get_sil_r2(jsd_t* self, uint16_t slave_id,
                                      uint16_t subindex, uint16_t app_id) {
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  if ((subindex >= 1 && subindex <= config->epd_sil.sil_r2_inputs_num) ||
      (subindex >= JSD_EPD_SIL_R2_OUTPUTS_SUBINDEX_OFFSET &&
       subindex <= ((JSD_EPD_SIL_R2_OUTPUTS_SUBINDEX_OFFSET - 1) +
                    config->epd_sil.sil_r2_outputs_num))) {
    ERROR("R2[%u] is already mapped to a PDO.", subindex);
    return;
  }
  jsd_sdo_get_param_async(self, slave_id, 0x22F4, subindex, JSD_SDO_DATA_DOUBLE,
                          app_id);
}

void jsd_epd_sil_async_sdo_set_drive_position(jsd_t* self, uint16_t slave_id,
                                              double   position,
                                              uint16_t app_id) {
  jsd_epd_async_sdo_set_drive_position_impl(self, slave_id, position, app_id);
}

void jsd_epd_sil_async_sdo_set_unit_mode(jsd_t* self, uint16_t slave_id,
                                         int16_t mode, uint16_t app_id) {
  jsd_epd_async_sdo_set_unit_mode_impl(self, slave_id, mode, app_id);
}

void jsd_epd_sil_async_sdo_set_ctrl_gain_scheduling_mode(
    jsd_t* self, uint16_t slave_id, jsd_elmo_gain_scheduling_mode_t mode,
    uint16_t app_id) {
  jsd_epd_async_sdo_set_ctrl_gain_scheduling_mode_impl(self, slave_id, mode,
                                                       app_id);
}

const char* jsd_epd_sil_fault_code_to_string(jsd_epd_fault_code_t fault_code) {
  return jsd_epd_fault_code_to_string_impl(fault_code);
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_epd_sil_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  assert(self->ecx_context.slavelist[slave_id].eep_man == JSD_ELMO_VENDOR_ID);

  ec_slavet* slave = &self->ecx_context.slavelist[slave_id];

  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  config->PO2SO_success      = false;

  MSG("EPD-SIL[%u] is configured with %d R1 inputs, %d R2 inputs, %d R1 "
      "outputs, and %d R2 outputs.",
      slave_id, config->epd_sil.sil_r1_inputs_num,
      config->epd_sil.sil_r2_inputs_num, config->epd_sil.sil_r1_outputs_num,
      config->epd_sil.sil_r2_outputs_num);

  // The following disables Complete Access (CA) and was needed in Gold drives
  // to make PDO mapping work.
  // TODO(dloret): Check if disabling CA is really necessary for Platinum
  // drives.
  slave->CoEdetails &= ~ECT_COEDET_SDOCA;

  slave->PO2SOconfigx = jsd_epd_sil_PO2SO_config;

  jsd_epd_sil_private_state_t* state = &self->slave_states[slave_id].epd_sil;
  state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_DISABLED;
  state->last_requested_mode_of_operation = state->requested_mode_of_operation;
  state->last_reset_time                  = 0;

  state->motor_rated_current = config->epd_sil.continuous_current_limit * 1000;
  if (state->motor_rated_current == 0) {
    ERROR("continuous_current_limit not set on EPD-SIL[%d]", slave_id);
    return false;
  }
  jsd_epd_sil_set_peak_current(self, slave_id,
                               config->epd_sil.peak_current_limit);

  state->pub.fault_code      = JSD_EPD_FAULT_OKAY;
  state->pub.emcy_error_code = 0;

  return true;
}

int jsd_epd_sil_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(jsd_epd_sil_product_code_is_compatible(
      ecx_context->slavelist[slave_id].eep_id));

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within the ecx_context and extract it here.
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;
  jsd_slave_config_t* config = &slave_configs[slave_id];

  if (!jsd_epd_sil_config_PDO_mapping(ecx_context, slave_id, config)) {
    ERROR("Failed to map PDO parameters on EPD-SIL slave %u", slave_id);
    return 0;
  }

  if (!jsd_epd_sil_config_COE_params(ecx_context, slave_id, config)) {
    ERROR("Failed to set COE parameters on EPD-SIL slave %u", slave_id);
    return 0;
  }

  if (!jsd_epd_sil_config_LC_params(ecx_context, slave_id, config)) {
    ERROR("Failed to set LC parameters on EPD-SIL slave %u", slave_id);
    return 0;
  }

  config->PO2SO_success = true;
  SUCCESS("EPD-SIL[%d] drive parameters successfully configured and verified",
          slave_id);
  return 1;
}

int jsd_epd_sil_config_PDO_mapping(ecx_contextt* ecx_context, uint16_t slave_id,
                                   jsd_slave_config_t* config) {
  MSG_DEBUG("Attempting to map custom EPD-SIL PDOs...");

  // RxPDO related variables
  uint16_t rpdo_mapping_parameters[]       = {0x1600, 0x1601, 0x1602, 0x1603};
  uint16_t rpdo_default_mapping_elements[] = {
      0x0010, 0x6040,  // controlword
      0x0010, 0x6073,  // max_current
      0x0008, 0x6060,  // mode_of_operation
  };
  int rpdo_default_variables_num =
      JSD_EPD_SIL_ARRAY_SIZE(rpdo_default_mapping_elements) / 2;

  // TxPDO related variables
  uint16_t tpdo_mapping_parameters[]       = {0x1A00, 0x1A01, 0x1A02, 0x1A03};
  uint16_t tpdo_default_mapping_elements[] = {
      0x0010, 0x6041,  // statusword
      0x0120, 0x3607,  // status_register_1
      0x0220, 0x3607,  // status_register_2
      0x0008, 0x6061,  // mode_of_operation_display
      0x0020, 0x6064,  // actual_position
      0x0020, 0x6069,  // velocity_actual_value
      0x0010, 0x6078,  // current_actual_value
  };
  int tpdo_default_variables_num =
      JSD_EPD_SIL_ARRAY_SIZE(tpdo_default_mapping_elements) / 2;

  // Check for errors before attempting any PDO mapping

  // Check for errors related to RxPDO mapping
  size_t rpdo_total_size = sizeof(jsd_epd_sil_rxpdo_data_t) +
                           (config->epd_sil.sil_r1_inputs_num * 4) +
                           (config->epd_sil.sil_r2_inputs_num * 8);
  if (rpdo_total_size > JSD_EPD_MAX_BYTES_PDO_CHANNEL) {
    ERROR(
        "Total size of RPDO mapped objects (%zu bytes) must be less than the "
        "corresponding PDO channel's limit (%d bytes).",
        rpdo_total_size, JSD_EPD_MAX_BYTES_PDO_CHANNEL);
    return 0;
  }
  size_t rpdo_total_objects = rpdo_default_variables_num +
                              config->epd_sil.sil_r1_inputs_num +
                              config->epd_sil.sil_r2_inputs_num;
  size_t rpdo_total_objects_limit =
      JSD_EPD_SIL_ARRAY_SIZE(rpdo_mapping_parameters) *
      JSD_EPD_SIL_MAX_OBJS_PER_PDO_MAPPING_PARAMETER;
  if (rpdo_total_objects > rpdo_total_objects_limit) {
    ERROR(
        "Number of RPDO mapped objects (%zu) must be less than the limit on "
        "RPDO mapped objects (%zu).",
        rpdo_total_objects, rpdo_total_objects_limit);
    return 0;
  }

  // Check for errors related to TxPDO mapping
  size_t tpdo_total_size = sizeof(jsd_epd_sil_txpdo_data_t) +
                           (config->epd_sil.sil_r1_outputs_num * 4) +
                           (config->epd_sil.sil_r2_outputs_num * 8);
  if (tpdo_total_size > JSD_EPD_MAX_BYTES_PDO_CHANNEL) {
    ERROR(
        "Total size of TPDO mapped objects (%zu bytes) must be less than the "
        "corresponding PDO channel's limit (%d bytes).",
        tpdo_total_size, JSD_EPD_MAX_BYTES_PDO_CHANNEL);
    return 0;
  }
  size_t tpdo_total_objects = tpdo_default_variables_num +
                              config->epd_sil.sil_r1_outputs_num +
                              config->epd_sil.sil_r2_outputs_num;
  size_t tpdo_total_objects_limit =
      JSD_EPD_SIL_ARRAY_SIZE(tpdo_mapping_parameters) *
      JSD_EPD_SIL_MAX_OBJS_PER_PDO_MAPPING_PARAMETER;
  if (tpdo_total_objects > tpdo_total_objects_limit) {
    ERROR(
        "Number of TPDO mapped objects (%zu) must be less than the limit on "
        "TPDO mapped objects (%zu).",
        tpdo_total_objects, tpdo_total_objects_limit);
    return 0;
  }

  // Perform RxPDO Mapping
  if (jsd_epd_config_PDO_mapping_helper(
          ecx_context, slave_id, rpdo_mapping_parameters,
          rpdo_default_mapping_elements, rpdo_default_variables_num,
          config->epd_sil.sil_r1_inputs_num, config->epd_sil.sil_r2_inputs_num,
          true) == 0) {
    ERROR("RxPDO mapping failed.");
    return 0;
  }
  MSG_DEBUG("RxPDO size: %zu Bytes", rpdo_total_size);

  // Perform TxPDO Mapping
  if (jsd_epd_config_PDO_mapping_helper(
          ecx_context, slave_id, tpdo_mapping_parameters,
          tpdo_default_mapping_elements, tpdo_default_variables_num,
          config->epd_sil.sil_r1_outputs_num,
          config->epd_sil.sil_r2_outputs_num, false) == 0) {
    ERROR("TxPDO mapping failed.");
    return 0;
  }
  MSG_DEBUG("TxPDO size: %zu Bytes", tpdo_total_size);

  return 1;
}

int jsd_epd_config_PDO_mapping_helper(ecx_contextt* ecx_context,
                                      uint16_t      slave_id,
                                      uint16_t      pdo_mapping_parameters[],
                                      uint16_t pdo_default_mapping_elements[],
                                      int      pdo_default_variables_num,
                                      int      sil_r1_variables_num,
                                      int sil_r2_variables_num, bool is_rxpdo) {
  // Index variables to iterate through the function parameter arrays
  int pdo_mapping_parameters_idx       = 0;
  int pdo_default_mapping_elements_idx = 0;
  // The 'unmapped' suffix refers to variables yet to be mapped by this
  // function.
  int pdo_default_variables_unmapped = pdo_default_variables_num;
  int sil_r1_variables_unmapped      = sil_r1_variables_num;
  int sil_r2_variables_unmapped      = sil_r2_variables_num;
  // By convention, R1 variables start at subindex 1 for RxPDO channel (SIL
  // inputs), and subindex 129 for TxPDO channel (SIL outputs).
  int sil_r1_variables_subindex =
      is_rxpdo ? 1 : JSD_EPD_SIL_R1_OUTPUTS_SUBINDEX_OFFSET;
  // By convention, R2 variables start at subindex 1 for RxPDO channel (SIL
  // inputs), and subindex 65 for TxPDO channel (SIL outputs).
  int sil_r2_variables_subindex =
      is_rxpdo ? 1 : JSD_EPD_SIL_R2_OUTPUTS_SUBINDEX_OFFSET;

  ////////// Set the PDO mapping parameter objects //////////

  // Each iteration of the while loop creates the mapping parameter record for
  // one of the mapping parameters contained in pdo_mapping_parameters.
  while ((pdo_default_variables_unmapped + sil_r1_variables_unmapped +
          sil_r2_variables_unmapped) > 0) {
    // Objects to be mapped in the record of the current loop iteration
    int objects_mapped = jsd_epd_sil_min(
        JSD_EPD_SIL_MAX_OBJS_PER_PDO_MAPPING_PARAMETER,
        pdo_default_variables_unmapped + sil_r1_variables_unmapped +
            sil_r2_variables_unmapped);
    // Allocate the array to contain the record. The structure of the record is
    // described in sections 5.5 and 5.6 of the Platinum Administrative Guide.
    // The first array entry is the total number of variables mapped in the
    // record. The following array entries correspond to the mapped variables.
    // Each variable spans two entries. The first entry of the pair encodes the
    // size and object subindex of the variable. The second entry corresponds to
    // the object index of the variable.
    uint16_t pdo_mapping_parameter_record[objects_mapped * 2 + 1];
    pdo_mapping_parameter_record[0] = objects_mapped;

    // Each iteration of the for loop fills in the mapping record with the
    // entries that correspond to one variable (two entries).
    for (int i = 0; i < objects_mapped; ++i) {
      int entry_start_idx = (i * 2) + 1;
      // Default variables are entered first, then R1 variables, and finally R2
      // variables.
      if (pdo_default_variables_unmapped > 0) {
        pdo_mapping_parameter_record[entry_start_idx] =
            pdo_default_mapping_elements[pdo_default_mapping_elements_idx];
        pdo_mapping_parameter_record[entry_start_idx + 1] =
            pdo_default_mapping_elements[++pdo_default_mapping_elements_idx];
        ++pdo_default_mapping_elements_idx;
        --pdo_default_variables_unmapped;
      } else if (sil_r1_variables_unmapped > 0) {
        // R1 variables are 32-bit integers (0x0020).
        pdo_mapping_parameter_record[entry_start_idx] =
            0x0020 + 0x0100 * sil_r1_variables_subindex;
        // Dictionary object 0x22F3 corresponds to R1 variables.
        pdo_mapping_parameter_record[entry_start_idx + 1] = 0x22F3;
        ++sil_r1_variables_subindex;
        --sil_r1_variables_unmapped;
      } else if (sil_r2_variables_unmapped > 0) {
        // R2 variables are 64-bit floating point numbers (0x0040).
        pdo_mapping_parameter_record[entry_start_idx] =
            0x0040 + 0x0100 * sil_r2_variables_subindex;
        // Dictionary object 0x22F4 corresponds to R2 variables.
        pdo_mapping_parameter_record[entry_start_idx + 1] = 0x22F4;
        ++sil_r2_variables_subindex;
        --sil_r2_variables_unmapped;
      }
    }

    for (unsigned int i = 0;
         i < JSD_EPD_SIL_ARRAY_SIZE(pdo_mapping_parameter_record); ++i) {
      MSG_DEBUG("pdo_mapping_parameter_record[%d] = %X", i,
                pdo_mapping_parameter_record[i]);
    }

    // Associate the just created record to the mapping parameter of the current
    // iteration
    if (!jsd_sdo_set_ca_param_blocking(
            ecx_context, slave_id,
            pdo_mapping_parameters[pdo_mapping_parameters_idx], 0x00,
            sizeof(pdo_mapping_parameter_record),
            &pdo_mapping_parameter_record)) {
      return 0;
    }

    ++pdo_mapping_parameters_idx;
  }

  ////////// Set PDO assign object //////////

  // After the while loop, pdo_mapping_parameters_idx corresponds to the total
  // number of mapping parameters set.
  uint16_t pdo_assign_array[1 + pdo_mapping_parameters_idx];
  pdo_assign_array[0] = pdo_mapping_parameters_idx;
  for (int i = 1; i <= pdo_mapping_parameters_idx; ++i) {
    pdo_assign_array[i] = pdo_mapping_parameters[i - 1];
  }
  for (unsigned int i = 0; i < JSD_EPD_SIL_ARRAY_SIZE(pdo_assign_array); ++i) {
    MSG_DEBUG("pdo_assign_array[%d] = %x", i, pdo_assign_array[i]);
  }
  uint16_t pdo_assign_object_index = is_rxpdo ? 0x1C12 : 0x1C13;
  if (!jsd_sdo_set_ca_param_blocking(
          ecx_context, slave_id, pdo_assign_object_index, 0x00,
          sizeof(pdo_assign_array), &pdo_assign_array)) {
    return 0;
  }

  return 1;
}

int jsd_epd_sil_config_COE_params(ecx_contextt* ecx_context, uint16_t slave_id,
                                  jsd_slave_config_t* config) {
  // TODO(dloret): original code checks that PROF_POS mode is supported. I might
  // want to switch to JSD_EPD_MODE_OF_OPERATION_PROF_TORQUE as default mode to
  // avoid this.

  // Put drive in PROF_POS mode by default
  int8_t controlword = JSD_EPD_MODE_OF_OPERATION_PROF_POS;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x6060, 0x00,
                                  JSD_SDO_DATA_I8, &controlword)) {
    return 0;
  }

  // Set Quick Stop option code
  // TODO(dloret): should Quick Stop deceleration (0x6085) be set too?
  int16_t quick_stop_opt_code =
      2;  // Slow down on quick-stop ramp and go to SWITCH ON DISABLED state
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x605A, 0,
                                  JSD_SDO_DATA_I16, &quick_stop_opt_code)) {
    return 0;
  }

  // Set motor rated current equal to the continuous current limit parameter
  uint32_t motor_rated_current =
      config->epd_sil.continuous_current_limit * 1000.0;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x6075, 0,
                                  JSD_SDO_DATA_U32, &motor_rated_current)) {
    return 0;
  }

  return 1;
}

int jsd_epd_sil_config_LC_params(ecx_contextt* ecx_context, uint16_t slave_id,
                                 jsd_slave_config_t* config) {
  // TODO(dloret): Verify the types of the corresponding data objects

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("PL"), 2,
          JSD_SDO_DATA_FLOAT, &config->epd_sil.peak_current_time)) {
    return 0;
  }

  // Note that the maximum current limit is also mapped to the RxPDO.
  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("PL"), 1,
          JSD_SDO_DATA_FLOAT, &config->epd_sil.peak_current_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("CL"), 1,
          JSD_SDO_DATA_FLOAT, &config->epd_sil.continuous_current_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("CL"), 2,
          JSD_SDO_DATA_FLOAT, &config->epd_sil.motor_stuck_current_level_pct)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("CL"), 3,
          JSD_SDO_DATA_FLOAT,
          &config->epd_sil.motor_stuck_velocity_threshold)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("CL"), 4,
          JSD_SDO_DATA_FLOAT, &config->epd_sil.motor_stuck_timeout)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("HL"), 2,
          JSD_SDO_DATA_DOUBLE, &config->epd_sil.over_speed_threshold)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("LL"), 3,
          JSD_SDO_DATA_DOUBLE, &config->epd_sil.low_position_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("HL"), 3,
          JSD_SDO_DATA_DOUBLE, &config->epd_sil.high_position_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("BP"), 1,
          JSD_SDO_DATA_I16, &config->epd_sil.brake_engage_msec)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("BP"), 2,
          JSD_SDO_DATA_I16, &config->epd_sil.brake_disengage_msec)) {
    return 0;
  }

  // Verify startup parameters

  // Verify checksum from the drive matches checksum recorded in configuration
  uint64_t crc = 0;
  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id,
                                  jsd_epd_sil_lc_to_do("CZ"), 1,
                                  JSD_SDO_DATA_U64, &crc)) {
    return 0;
  }
  MSG("EPD-SIL[%d] CRC = %lu", slave_id, crc);
  if (config->epd_sil.crc == 0) {
    MSG("EPD-SIL[%d] drive parameter CRC check overridden", slave_id);
  } else {
    if (crc != config->epd_sil.crc) {
      ERROR(
          "EPD-SIL[%d] CRC mismatch - YAML value: %u, actual drive value: %lu",
          slave_id, config->epd_sil.crc, crc);
      return 0;
    }
  }

  // Verify current limits
  float drive_max_current = 0;
  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id,
                                  jsd_epd_sil_lc_to_do("MC"), 1,
                                  JSD_SDO_DATA_FLOAT, &drive_max_current)) {
    return 0;
  }
  MSG("EPD-SIL[%d] Drive Maximum Current is %f A", slave_id, drive_max_current);

  if (config->epd_sil.peak_current_limit > drive_max_current) {
    // TODO(dloret): Check if the drive can even allow to set PL[1] if it is
    // greater than MC[1]. PL[1] is set above.
    ERROR(
        "EPD-SIL[%d] Peak Current (%f) cannot exceed Drive Maximum Current "
        "(%f)",
        slave_id, config->epd_sil.peak_current_limit, drive_max_current);
    return 0;
  }

  if (config->epd_sil.continuous_current_limit >
      config->epd_sil.peak_current_limit) {
    // TODO(dloret): this would actually disable CL[1] and is valid. Investigate
    // what is the implication of disabling CL[1].
    ERROR(
        "EPD-SIL[%d] Continous Current (%f) should not exceed Peak Current "
        "(%f)",
        slave_id, config->epd_sil.continuous_current_limit,
        config->epd_sil.peak_current_limit);
    return 0;
  }

  // Display highest allowed control loop (UM[1]=1 -> current control loop,
  // UM[1]=2 -> velocity control loop, UM[1]=5 -> position control loop).
  int16_t um = 0;
  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id,
                                  jsd_epd_sil_lc_to_do("UM"), 1,
                                  JSD_SDO_DATA_I16, &um)) {
    return 0;
  }
  MSG("EPD-SIL[%d] UM[1] = %d", slave_id, um);

  return 1;
}

void jsd_epd_sil_update_state_from_PDO_data(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_epd_sil_private_state_t* state = &self->slave_states[slave_id].epd_sil;

  state->last_state_machine_state = state->pub.actual_state_machine_state;

  state->pub.actual_position = state->default_txpdo.actual_position;
  state->pub.actual_velocity = state->default_txpdo.velocity_actual_value;
  state->pub.actual_current =
      (double)state->default_txpdo.current_actual_value *
      state->motor_rated_current / 1e6;

  state->pub.actual_mode_of_operation =
      state->default_txpdo.mode_of_operation_display;
  // TODO(dloret): EGD code prints a change of mode of operation here.

  // Handle Statusword
  state->pub.actual_state_machine_state =
      state->default_txpdo.statusword & JSD_EPD_STATE_MACHINE_STATE_BITMASK;
  if (state->pub.actual_state_machine_state !=
      state->last_state_machine_state) {
    MSG("EPD-SIL[%d] actual State Machine State changed to %s (0x%x)", slave_id,
        jsd_elmo_state_machine_state_to_string(
            state->pub.actual_state_machine_state),
        state->pub.actual_state_machine_state);

    if (state->pub.actual_state_machine_state ==
        JSD_ELMO_STATE_MACHINE_STATE_FAULT) {
      // TODO(dloret): Check if setting state->new_reset to false like in EGD
      // code is actually needed. Commands are handled after reading functions.
      state->fault_real_time = jsd_time_get_time_sec();
      state->fault_mono_time = jsd_time_get_mono_time_sec();

      jsd_sdo_signal_emcy_check(self);
    }
  }

  state->pub.warning        = state->default_txpdo.statusword >> 7 & 0x01;
  state->pub.target_reached = state->default_txpdo.statusword >> 10 & 0x01;

  // Handle Status Register
  state->pub.servo_enabled = state->default_txpdo.status_register_1 >> 4 & 0x01;
  state->fault_occured_when_enabled =
      state->default_txpdo.status_register_1 >> 6 & 0x01;
  // TODO(dloret): Double check this is a proper way to check STO status.
  state->pub.sto_engaged =
      !((state->default_txpdo.status_register_1 >> 25 & 0x01) &
        (state->default_txpdo.status_register_1 >> 26 & 0x01));
  state->pub.motor_on   = state->default_txpdo.status_register_1 >> 22 & 0x01;
  state->pub.in_motion  = state->default_txpdo.status_register_1 >> 23 & 0x01;
  state->pub.hall_state = state->default_txpdo.status_register_2 >> 0 & 0x07;

  // TODO(dloret): EGD code prints change in sto_engaged here.

  state->pub.cmd_max_current = (double)state->default_rxpdo.max_current *
                               state->motor_rated_current / 1e6;

  state->pub.sil_initialized =
      state->default_txpdo.status_register_2 >> 17 & 0x01;
  state->pub.sil_running = state->default_txpdo.status_register_2 >> 18 & 0x01;
  state->pub.sil_faulted = state->default_txpdo.status_register_2 >> 19 & 0x01;
  // R1 and R2 input/output arrays are populated in jsd_epd_read function.
}

void jsd_epd_sil_process_state_machine(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_epd_sil_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_epd_sil_private_state_t* state = &self->slave_states[slave_id].epd_sil;

  switch (state->pub.actual_state_machine_state) {
    case JSD_ELMO_STATE_MACHINE_STATE_NOT_READY_TO_SWITCH_ON:
      // This case should never execute because it is an internal initial state
      // that cannot be monitored by the host.
      break;
    case JSD_ELMO_STATE_MACHINE_STATE_SWITCH_ON_DISABLED:
      // Transition to READY TO SWITCH ON
      state->default_rxpdo.controlword =
          JSD_EPD_STATE_MACHINE_CONTROLWORD_SHUTDOWN;
      break;
    case JSD_ELMO_STATE_MACHINE_STATE_READY_TO_SWITCH_ON:
      // Transition to SWITCHED ON
      state->default_rxpdo.controlword =
          JSD_EPD_STATE_MACHINE_CONTROLWORD_SWITCH_ON;
      break;
    case JSD_ELMO_STATE_MACHINE_STATE_SWITCHED_ON:
      // Startup, a fault, or the completion of a halt command (i.e. Quick Stop)
      // eventually land in this state. Transition to OPERATION ENABLED if a
      // reset command has been received.
      if (state->new_reset) {
        state->default_rxpdo.controlword =
            JSD_EPD_STATE_MACHINE_CONTROLWORD_ENABLE_OPERATION;
        state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_PROF_POS;
        state->default_rxpdo.mode_of_operation =
            state->requested_mode_of_operation;
        state->new_reset = false;
      }
      break;
    case JSD_ELMO_STATE_MACHINE_STATE_OPERATION_ENABLED:
      state->pub.fault_code      = JSD_EPD_FAULT_OKAY;
      state->pub.emcy_error_code = 0;

      // Handle halt (Quick Stop)
      if (state->new_halt_command) {
        // Make sure OPERATON ENABLED will not be entered immediately after the
        // Quick Stop if a reset command was issued together with the halt.
        state->new_reset = false;
        // Invoke the Quick Stop function
        // TODO(dloret): EGD code overwrites previous controlword, maybe to not
        // change the mode of operation bits. It does not seem to me that is
        // necessary.
        state->default_rxpdo.controlword =
            JSD_EPD_STATE_MACHINE_CONTROLWORD_QUICK_STOP;
        state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_PROF_POS;
        state->default_rxpdo.mode_of_operation =
            state->requested_mode_of_operation;
        break;
      }
      break;
    case JSD_ELMO_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE:
      // No-op. Since the Quick Stop Option Code (0x605A) is set to 2, the drive
      // transitions into SWITCH ON DISABLED at completion of the Quick Stop.
      // TODO(dloret): If this does not work, try setting the controlword to
      // JSD_EPD_STATE_MACHINE_CONTROLWORD_DISABLE_VOLTAGE which includes QUICK
      // STOP ACTIVE -> SWITCH ON DISABLED.
      break;
    case JSD_ELMO_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE:
      // No-op. Transition from FAULT REACTION ACTIVE to FAULT happens
      // automatically at completion of the fault reaction stop.
      break;
    case JSD_ELMO_STATE_MACHINE_STATE_FAULT:;  // Semicolon needed to allow
                                               // placement of declarations
                                               // after label
      jsd_error_cirq_t* error_cirq = &self->slave_errors[slave_id];
      ec_errort         error;
      // Try to recover the EMCY code before transitioning out of FAULT.
      // Hopefully, the corresponding EMCY code has a timestamp greater than
      // when the driver detected the EPD's transition into the FAULT state.
      bool error_found    = false;
      int  num_error_pops = 0;
      // TODO(dloret): Might want to use the non-mutex interface of
      // jsd_error_cirq and incorporate a dedicated mutex for access to the
      // queue here and in the error handling (e.g. pushing errors).
      while (num_error_pops < JSD_EPD_SIL_MAX_ERROR_POPS_PER_CYCLE &&
             jsd_error_cirq_pop(error_cirq, &error)) {
        if (ectime_to_sec(error.Time) > state->fault_real_time) {
          // Might want to handle other types of errors too in the future.
          if (error.Etype == EC_ERR_TYPE_EMERGENCY) {
            state->pub.emcy_error_code = error.ErrorCode;
            state->pub.fault_code = jsd_epd_get_fault_code_from_ec_error(error);

            // TODO(dloret): Remove printing to not affect real-time guarantees.
            ERROR("EPD-SIL[%d] EMCY code: 0x%X, fault description: %s",
                  error.Slave, state->pub.emcy_error_code,
                  jsd_epd_sil_fault_code_to_string(state->pub.fault_code));

            // Transition to SWITCHED ON DISABLED
            state->default_rxpdo.controlword =
                JSD_EPD_STATE_MACHINE_CONTROLWORD_FAULT_RESET;

            error_found = true;
            break;  // break from the while loop
          }
        }
        ++num_error_pops;
      }
      // If the error has not arrived within 1 second, transition out of FAULT
      // because it might never arrive (e.g. error at startup).
      if (!error_found &&
          jsd_time_get_mono_time_sec() > (1.0 + state->fault_mono_time)) {
        // TODO(dloret): Remove printing to not affect real-time guarantees.
        WARNING("EPD-SIL[%d] in FAULT state but new EMCY code has not arrived",
                slave_id);

        state->pub.emcy_error_code = 0xFFFF;
        state->pub.fault_code      = JSD_EPD_FAULT_UNKNOWN;

        // Transition to SWITCHED ON DISABLED
        state->default_rxpdo.controlword =
            JSD_EPD_STATE_MACHINE_CONTROLWORD_FAULT_RESET;
      }
      break;
    default:
      ERROR(
          "EPD-SIL[%d] Unknown state machine state: 0x%x. This should never "
          "happen. Exiting.",
          slave_id, state->pub.actual_state_machine_state);
      assert(0);
  }
  state->new_halt_command = false;
}

bool jsd_epd_sil_product_code_is_compatible(uint32_t product_code) {
  return jsd_epd_product_code_is_compatible_impl(product_code);
}
