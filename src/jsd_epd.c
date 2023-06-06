#include "jsd/jsd_epd.h"

#include <assert.h>
#include <string.h>

#include "ethercat.h"
#include "jsd/jsd.h"
#include "jsd/jsd_elmo_common.h"
#include "jsd/jsd_sdo.h"

#define JSD_EPD_MAX_ERROR_POPS_PER_CYCLE (5)
#define JSD_EPD_MAX_BYTES_PDO_CHANNEL (128)

// Pair of Elmo letter command and corresponding object dictionary index
typedef struct {
  char*    lc_chars;
  uint16_t do_index;
} jsd_epd_lc_pair_t;

// Lookup table to map letter command characters to the corresponding object
// dictionary index.
// IMPORTANT! This table must be kept in alphabetical order so that the lookup
// function works.
static const jsd_epd_lc_pair_t jsd_epd_lc_lookup_table[] = {
    {"AC", 0x300C},
    {"BP", 0x303D},  // TODO(dloret): verify this is the right
                     // index. Documentation shows multiple
                     // indeces.
    {"CA", 0x3052},
    {"CL", 0x305D},
    {"CZ", 0x306B},
    {"DC", 0x3078},
    {"ER", 0x30AB},
    {"GS", 0x30F4},
    {"HL", 0x3111},
    {"LL", 0x31A1},
    {"MC", 0x31BC},
    {"PL", 0x3231},
    {"PX", 0x323D},
    {"SF", 0x3297},
    {"UM", 0x32E6},
};

static int jsd_epd_compare_lc_keys(const void* lhs, const void* rhs) {
  const jsd_epd_lc_pair_t* const l = lhs;
  const jsd_epd_lc_pair_t* const r = rhs;

  return strcmp(l->lc_chars, r->lc_chars);
}

/****************************************************
 * Public functions
 ****************************************************/

uint16_t jsd_epd_lc_to_do(char letter_command[2]) {
  jsd_epd_lc_pair_t  key        = {.lc_chars = letter_command};
  jsd_epd_lc_pair_t* found_pair = bsearch(
      &key, jsd_epd_lc_lookup_table,
      sizeof(jsd_epd_lc_lookup_table) / sizeof(jsd_epd_lc_lookup_table[0]),
      sizeof(jsd_epd_lc_lookup_table[0]), jsd_epd_compare_lc_keys);
  return found_pair ? found_pair->do_index : 0x0000;
}

const jsd_epd_state_t* jsd_epd_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);
  return &self->slave_states[slave_id].epd.pub;
}

void jsd_epd_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  // Copy TxPDO data from SOEM's IOmap
  assert(sizeof(jsd_epd_txpdo_data_t) ==
         self->ecx_context.slavelist[slave_id].Ibytes);
  memcpy(&self->slave_states[slave_id].epd.txpdo,
         self->ecx_context.slavelist[slave_id].inputs,
         self->ecx_context.slavelist[slave_id].Ibytes);

  jsd_epd_update_state_from_PDO_data(self, slave_id);
}

void jsd_epd_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_process_state_machine(self, slave_id);

  // Copy RxPDO data into SOEM's IOmap
  assert(sizeof(jsd_epd_rxpdo_data_t) ==
         self->ecx_context.slavelist[slave_id].Obytes);
  memcpy(self->ecx_context.slavelist[slave_id].outputs,
         &self->slave_states[slave_id].epd.rxpdo,
         self->ecx_context.slavelist[slave_id].Obytes);
}

void jsd_epd_reset(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  double now = jsd_time_get_mono_time_sec();

  if ((now - self->slave_states[slave_id].epd.last_reset_time) >
      JSD_EPD_RESET_DERATE_SEC) {
    self->slave_states[slave_id].epd.new_reset       = true;
    self->slave_states[slave_id].epd.last_reset_time = now;

    // TODO(dloret): EGD code clears fault_code/emcy_error_code here. However,
    // the clearing is also done when entering the OPERATION_ENABLED state. It
    // seems only one is needed and clearing in OPERATION_ENABLED is more in
    // line with only 2 states visible for user: error and non-error (i.e.
    // OPERATION_ENABLED).
  } else {
    // TODO(dloret): Remove printing to not affect real-time guarantees.
    WARNING(
        "EPD Reset Derate Protection feature is preventing reset, ignoring "
        "request");
  }
}

void jsd_epd_halt(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  self->slave_states[slave_id].epd.new_halt_command = true;
}

void jsd_epd_set_gain_scheduling_index(jsd_t* self, uint16_t slave_id,
                                       bool     lsb_byte,
                                       uint16_t gain_scheduling_index) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  if (gain_scheduling_index < 1 || gain_scheduling_index > 63) {
    ERROR("The provided gain scheduling index %d is out of range [1,63]",
          gain_scheduling_index);
    return;
  }

  jsd_epd_private_state_t* state   = &self->slave_states[slave_id].epd;
  uint16_t                 bitmask = 0x00FF;
  if (lsb_byte) {
    state->rxpdo.gain_scheduling_index =
        (state->rxpdo.gain_scheduling_index & (bitmask << 8)) |
        gain_scheduling_index;
  } else {
    state->rxpdo.gain_scheduling_index =
        (state->rxpdo.gain_scheduling_index & bitmask) |
        (gain_scheduling_index << 8);
  }
}

void jsd_epd_set_digital_output(jsd_t* self, uint16_t slave_id, uint8_t index,
                                uint8_t output) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);
  assert(index > 0 && index <= JSD_EPD_NUM_DIGITAL_OUTPUTS);

  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;
  if (output > 0) {
    state->rxpdo.digital_outputs |= (0x01 << (15 + index));
  } else {
    state->rxpdo.digital_outputs &= ~(0x01 << (15 + index));
  }
}

void jsd_epd_set_peak_current(jsd_t* self, uint16_t slave_id,
                              double peak_current) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;

  state->rxpdo.max_current = peak_current * 1e6 / state->motor_rated_current;
}

void jsd_epd_set_motion_command_csp(
    jsd_t* self, uint16_t slave_id,
    jsd_elmo_motion_command_csp_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_private_state_t* state     = &self->slave_states[slave_id].epd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_CSP;
  state->motion_command.csp          = motion_command;
}

void jsd_epd_set_motion_command_csv(
    jsd_t* self, uint16_t slave_id,
    jsd_elmo_motion_command_csv_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_private_state_t* state     = &self->slave_states[slave_id].epd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_CSV;
  state->motion_command.csv          = motion_command;
}

void jsd_epd_set_motion_command_cst(
    jsd_t* self, uint16_t slave_id,
    jsd_elmo_motion_command_cst_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_private_state_t* state     = &self->slave_states[slave_id].epd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_CST;
  state->motion_command.cst          = motion_command;
}

void jsd_epd_set_motion_command_prof_pos(
    jsd_t* self, uint16_t slave_id,
    jsd_elmo_motion_command_prof_pos_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_private_state_t* state     = &self->slave_states[slave_id].epd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_PROF_POS;
  state->motion_command.prof_pos     = motion_command;
}

void jsd_epd_set_motion_command_prof_vel(
    jsd_t* self, uint16_t slave_id,
    jsd_elmo_motion_command_prof_vel_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_private_state_t* state     = &self->slave_states[slave_id].epd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_PROF_VEL;
  state->motion_command.prof_vel     = motion_command;
}

void jsd_epd_set_motion_command_prof_torque(
    jsd_t* self, uint16_t slave_id,
    jsd_elmo_motion_command_prof_torque_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_private_state_t* state     = &self->slave_states[slave_id].epd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_PROF_TORQUE;
  state->motion_command.prof_torque  = motion_command;
}

void jsd_epd_async_sdo_set_drive_position(jsd_t* self, uint16_t slave_id,
                                          double position, uint16_t app_id) {
  jsd_sdo_set_param_async(self, slave_id, jsd_epd_lc_to_do("PX"), 1,
                          JSD_SDO_DATA_DOUBLE, &position, app_id);
}

void jsd_epd_async_sdo_set_unit_mode(jsd_t* self, uint16_t slave_id,
                                     int16_t mode, uint16_t app_id) {
  jsd_sdo_set_param_async(self, slave_id, jsd_epd_lc_to_do("UM"), 1,
                          JSD_SDO_DATA_I16, &mode, app_id);
}

void jsd_epd_async_sdo_set_ctrl_gain_scheduling_mode(
    jsd_t* self, uint16_t slave_id, jsd_elmo_gain_scheduling_mode_t mode,
    uint16_t app_id) {
  if (mode < 0 || mode > 68) {
    ERROR("Gain scheduling mode %d for the controller is not valid.", mode);
    return;
  }
  int64_t mode_i64 = mode;
  jsd_sdo_set_param_async(self, slave_id, jsd_epd_lc_to_do("GS"), 2,
                          JSD_SDO_DATA_I64, &mode_i64, app_id);
}

const char* jsd_epd_fault_code_to_string(jsd_epd_fault_code_t fault_code) {
  switch (fault_code) {
    case JSD_EPD_FAULT_OKAY:
      return "JSD_EPD_FAULT_OKAY";
    case JSD_EPD_FAULT_SHORT_PROTECTION:
      return "JSD_EPD_FAULT_SHORT_PROTECTION";
    case JSD_EPD_FAULT_UNDER_VOLTAGE:
      return "JSD_EPD_FAULT_UNDER_VOLTAGE";
    case JSD_EPD_FAULT_LOSS_OF_PHASE:
      return "JSD_EPD_FAULT_LOSS_OF_PHASE";
    case JSD_EPD_FAULT_OVER_VOLTAGE:
      return "JSD_EPD_FAULT_OVER_VOLTAGE";
    case JSD_EPD_FAULT_MOTOR_OVER_TEMPERATURE:
      return "JSD_EPD_FAULT_MOTOR_OVER_TEMPERATURE";
    case JSD_EPD_FAULT_DRIVE_OVER_TEMPERATURE:
      return "JSD_EPD_FAULT_DRIVE_OVER_TEMPERATURE";
    case JSD_EPD_FAULT_GANTRY_YAW_ERROR_LIMIT_EXCEEDED:
      return "JSD_EPD_FAULT_GANTRY_YAW_ERROR_LIMIT_EXCEEDED";
    case JSD_EPD_FAULT_EXTERNAL_INHIBIT_TRIGGERED:
      return "JSD_EPD_FAULT_EXTERNAL_INHIBIT_TRIGGERED";
    case JSD_EPD_FAULT_ADDITIONAL_ABORT_ACTIVE:
      return "JSD_EPD_FAULT_ADDITIONAL_ABORT_ACTIVE";
    case JSD_EPD_FAULT_VECTOR_ABORT:
      return "JSD_EPD_FAULT_VECTOR_ABORT";
    case JSD_EPD_FAULT_RPDO_FAILED:
      return "JSD_EPD_FAULT_RPDO_FAILED";
    case JSD_EPD_FAULT_MOTOR_STUCK:
      return "JSD_EPD_FAULT_MOTOR_STUCK";
    case JSD_EPD_FAULT_FEEDBACK_ERROR:
      return "JSD_EPD_FAULT_FEEDBACK_ERROR";
    case JSD_EPD_FAULT_HALL_MAIN_FEEDBACK_MISMATCH:
      return "JSD_EPD_FAULT_HALL_MAIN_FEEDBACK_MISMATCH";
    case JSD_EPD_FAULT_HALL_BAD_CHANGE:
      return "JSD_EPD_FAULT_HALL_BAD_CHANGE";
    case JSD_EPD_FAULT_COMMUTATION_PROCESS_FAIL:
      return "JSD_EPD_FAULT_COMMUTATION_PROCESS_FAIL";
    case JSD_EPD_FAULT_CAN_MESSAGE_LOST:
      return "JSD_EPD_FAULT_CAN_MESSAGE_LOST";
    case JSD_EPD_FAULT_SYNC_OR_FRAME_LOSS:
      return "JSD_EPD_FAULT_SYNC_OR_FRAME_LOSS";
    case JSD_EPD_FAULT_RECOVERED_FROM_BUS_OFF:
      return "JSD_EPD_FAULT_RECOVERED_FROM_BUS_OFF";
    case JSD_EPD_FAULT_ACCESS_NON_CONFIGURED_RPDO:
      return "JSD_EPD_FAULT_ACCESS_NON_CONFIGURED_RPDO";
    case JSD_EPD_FAULT_INCORRECT_RPDO_LENGTH:
      return "JSD_EPD_FAULT_INCORRECT_RPDO_LENGTH";
    case JSD_EPD_FAULT_PEAK_CURRENT_EXCEEDED:
      return "JSD_EPD_FAULT_PEAK_CURRENT_EXCEEDED";
    case JSD_EPD_FAULT_SPEED_TRACKING_ERROR:
      return "JSD_EPD_FAULT_SPEED_TRACKING_ERROR";
    case JSD_EPD_FAULT_SPEED_LIMIT_EXCEEDED:
      return "JSD_EPD_FAULT_SPEED_LIMIT_EXCEEDED";
    case JSD_EPD_FAULT_POSITION_TRACKING_ERROR:
      return "JSD_EPD_FAULT_POSITION_TRACKING_ERROR";
    case JSD_EPD_FAULT_POSITION_LIMIT_EXCEEDED:
      return "JSD_EPD_FAULT_POSITION_LIMIT_EXCEEDED";
    case JSD_EPD_FAULT_CAN_INTERPOLATED_MODE_EMERGENCY:
      return "JSD_EPD_FAULT_CAN_INTERPOLATED_MODE_EMERGENCY";
    case JSD_EPD_FAULT_CANNOT_START_MOTOR:
      return "JSD_EPD_FAULT_CANNOT_START_MOTOR";
    case JSD_EPD_FAULT_STO_ENGAGED:
      return "JSD_EPD_FAULT_STO_ENGAGED";
    case JSD_EPD_FAULT_MOTOR_DISABLE_COMMAND:
      return "JSD_EPD_FAULT_MOTOR_DISABLE_COMMAND";
    case JSD_EPD_FAULT_KINEMATICS_ERROR:
      return "JSD_EPD_FAULT_KINEMATICS_ERROR";
    case JSD_EPD_FAULT_GANTRY_MASTER_ERROR:
      return "JSD_EPD_FAULT_GANTRY_MASTER_ERROR";
    case JSD_EPD_FAULT_GANTRY_SLAVE_DISABLED:
      return "JSD_EPD_FAULT_GANTRY_SLAVE_DISABLED";
    case JSD_EPD_FAULT_GANTRY_ATTACHED_SLAVE_FAULT:
      return "JSD_EPD_FAULT_GANTRY_ATTACHED_SLAVE_FAULT";
    case JSD_EPD_FAULT_UNKNOWN:
      return "JSD_EPD_FAULT_UNKNOWN";
    default:
      return "Unknown Fault Code";
  }
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_epd_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man == JSD_ELMO_VENDOR_ID);
  assert(sizeof(jsd_epd_txpdo_data_t) <= JSD_EPD_MAX_BYTES_PDO_CHANNEL);
  assert(sizeof(jsd_epd_rxpdo_data_t) <= JSD_EPD_MAX_BYTES_PDO_CHANNEL);

  ec_slavet* slave = &self->ecx_context.slavelist[slave_id];

  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  config->PO2SO_success      = false;

  // The following disables Complete Access (CA) and was needed in Gold drives
  // to make PDO mapping work.
  // TODO(dloret): Check if disabling CA is really necessary for Platinum
  // drives.
  slave->CoEdetails &= ~ECT_COEDET_SDOCA;

  slave->PO2SOconfigx = jsd_epd_PO2SO_config;

  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;
  state->requested_mode_of_operation      = JSD_EPD_MODE_OF_OPERATION_DISABLED;
  state->last_requested_mode_of_operation = state->requested_mode_of_operation;
  state->last_reset_time         = 0;

  MSG_DEBUG("TxPDO size: %zu Bytes", sizeof(jsd_epd_txpdo_data_t));
  MSG_DEBUG("RxPDO size: %zu Bytes", sizeof(jsd_epd_rxpdo_data_t));

  state->motor_rated_current = config->epd.continuous_current_limit * 1000;
  if (state->motor_rated_current == 0) {
    ERROR("continuous_current_limit not set on EPD[%d]", slave_id);
    return false;
  }
  jsd_epd_set_peak_current(self, slave_id, config->epd.peak_current_limit);

  state->pub.fault_code      = JSD_EPD_FAULT_OKAY;
  state->pub.emcy_error_code = 0;

  state->setpoint_ack                  = 0;
  state->last_setpoint_ack             = 0;
  state->prof_pos_waiting_setpoint_ack = false;

  return true;
}

int jsd_epd_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within the ecx_context and extract it here.
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;
  jsd_slave_config_t* config = &slave_configs[slave_id];

  if (!jsd_epd_config_PDO_mapping(ecx_context, slave_id)) {
    ERROR("Failed to map PDO parameters on EPD slave %u", slave_id);
    return 0;
  }

  if (!jsd_epd_config_COE_params(ecx_context, slave_id, config)) {
    ERROR("Failed to set COE parameters on EPD slave %u", slave_id);
    return 0;
  }

  if (!jsd_epd_config_LC_params(ecx_context, slave_id, config)) {
    ERROR("Failed to set LC parameters on EPD slave %u", slave_id);
    return 0;
  }

  config->PO2SO_success = true;
  SUCCESS("EPD[%d] drive parameters successfully configured and verified",
          slave_id);
  return 1;
}

int jsd_epd_config_PDO_mapping(ecx_contextt* ecx_context, uint16_t slave_id) {
  MSG_DEBUG("Attempting to map custom EPD PDOs...");

  //////////////// RxPDO Mapping //////////////////////////
  uint16_t map_output_pdos_1602[] = {
      0x0008,          // Number of mapped parameters
      0x0020, 0x607A,  // target_position
      0x0020, 0x60FF,  // target_velocity
      0x0010, 0x6071,  // target_torque
      0x0020, 0x60B0,  // position_offset
      0x0020, 0x60B1,  // velocity_offset
      0x0010, 0x60B2,  // torque_offset
      0x0008, 0x6060,  // mode_of_operation
      0x0010, 0x6073,  // max_current
  };
  if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1602, 0x00,
                                     sizeof(map_output_pdos_1602),
                                     &map_output_pdos_1602)) {
    return 0;
  }

  uint16_t map_output_pdos_1603[] = {
      0x0007,          // Number of mapped parameters
      0x0120, 0x60FE,  // digital_outputs
      0x0010, 0x6040,  // controlword
      0x0020, 0x6081,  // profile_velocity
      0x0020, 0x6082,  // end_velocity
      0x0020, 0x6083,  // profile_accel
      0x0020, 0x6084,  // profile_decel
      0x0010, 0x36E0,  // gain_scheduling_index
  };
  if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1603, 0x00,
                                     sizeof(map_output_pdos_1603),
                                     &map_output_pdos_1603)) {
    return 0;
  }

  // TODO(dloret): Didn't we disable Complete Access somewhere else?
  uint16_t map_output_RxPDO[] = {0x0002, 0x1602, 0x1603};
  if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1C12, 0x00,
                                     sizeof(map_output_RxPDO),
                                     &map_output_RxPDO)) {
    return 0;
  }

  //////////////// TxPDO Mapping //////////////////////////
  uint16_t map_input_pdos_1a02[] = {
      0x0008,          // Number of mapped parameters
      0x0020, 0x6064,  // actual_position
      0x0020, 0x6069,  // velocity_actual_value
      0x0010, 0x6078,  // current_actual_value
      0x0008, 0x6061,  // mode_of_operation_display
      0x0020, 0x6079,  // dc_link_circuit_voltage
      0x0020, 0x3610,  // drive_temperature_deg_c
      0x0020, 0x60FD,  // digital_inputs
      0x0110, 0x2205,  // analog_input_1
  };
  if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1A02, 0x00,
                                     sizeof(map_input_pdos_1a02),
                                     &map_input_pdos_1a02)) {
    return 0;
  }

  uint16_t map_input_pdos_1a03[] = {
      0x0004,          // Number of mapped parameters
      0x0210, 0x2205,  // analog_input_2
      0x0120, 0x3607,  // status_register_1
      0x0220, 0x3607,  // status_register_2
      0x0010, 0x6041,  // statusword
  };
  if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1A03, 0x00,
                                     sizeof(map_input_pdos_1a03),
                                     &map_input_pdos_1a03)) {
    return 0;
  }

  uint16_t map_input_TxPDO[] = {0x0002, 0x1A02, 0x1A03};
  if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1C13, 0x00,
                                     sizeof(map_input_TxPDO),
                                     &map_input_TxPDO)) {
    return 0;
  }

  return 1;
}

int jsd_epd_config_COE_params(ecx_contextt* ecx_context, uint16_t slave_id,
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

  // Set relative motion to be relative to actual position
  // TODO(dloret): Latest Platinum's firmware has a bug with option 0x02
  // (relative to actual position). Until the issue is fixed, 0x01 (relative to
  // desired position, not target profiled position) will be used to have a
  // similar behavior.
  uint16_t pos_opt_code = 0x01;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x60F2, 0x00,
                                  JSD_SDO_DATA_U16, &pos_opt_code)) {
    return 0;
  }

  // Set interpolation time period.
  // Drive actually supports microseconds.
  uint8_t loop_period_ms = config->epd.loop_period_ms;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x60C2, 1,
                                  JSD_SDO_DATA_U8, &loop_period_ms)) {
    return 0;
  }

  // Set Extrapolation Cycles Timeout (5 cycles based on ECAT lib testing)
  // TODO(dloret): confirm whether object 0x2F75 remains unchanged for the
  // Platinum.
  int16_t extra_cycles = 5;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3675, 0,
                                  JSD_SDO_DATA_I16, &extra_cycles)) {
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
  uint32_t motor_rated_current = config->epd.continuous_current_limit * 1000.0;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x6075, 0,
                                  JSD_SDO_DATA_U32, &motor_rated_current)) {
    return 0;
  }

  // Set torque slope for profile torque commands
  uint32_t torque_slope = config->epd.torque_slope * 1e6 / motor_rated_current;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x6087, 0,
                                  JSD_SDO_DATA_U32, &torque_slope)) {
    return 0;
  }

  // Set maximum motor speed
  // First, get feedback counts per electrical cycle (e.g. encoder counts per
  // revolution) because the maximum motor speed parameter expects rpm units.
  int64_t ca_18;
  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("CA"),
                                  18, JSD_SDO_DATA_I64, &ca_18)) {
    return 0;
  }
  MSG("EPD[%d] read CA[18] = %ld counts per revolution", slave_id, ca_18);
  // Express maximum motor speed in rpm units.
  if (config->epd.max_motor_speed < 0.0) {
    ERROR(
        "EPD[%d] failed to set maximum motor speed (%lf). The parameter must "
        "not be negative.",
        slave_id, config->epd.max_motor_speed);
    return 0;
  }
  uint32_t max_motor_speed_rpm =
      (uint32_t)(config->epd.max_motor_speed / ca_18 * 60.0);
  MSG("EPD[%d] max_motor_speed_rpm = %u", slave_id, max_motor_speed_rpm);
  // Finally, set the maximum motor speed object.
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x6080, 0,
                                  JSD_SDO_DATA_U32, &max_motor_speed_rpm)) {
    return 0;
  }

  return 1;
}

int jsd_epd_config_LC_params(ecx_contextt* ecx_context, uint16_t slave_id,
                             jsd_slave_config_t* config) {
  // TODO(dloret): Verify the types of the corresponding data objects
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("AC"),
                                  1, JSD_SDO_DATA_DOUBLE,
                                  &config->epd.max_profile_accel)) {
    // TODO(dloret): EGD code warns about a minimum permissible profile
    // acceleration. Not sure if this applies to Platinum.
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("DC"),
                                  1, JSD_SDO_DATA_DOUBLE,
                                  &config->epd.max_profile_decel)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("QS"),
                                  1, JSD_SDO_DATA_DOUBLE,
                                  &config->epd.quick_stop_decel)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("SD"),
                                  1, JSD_SDO_DATA_DOUBLE,
                                  &config->epd.stop_decel)) {
    return 0;
  }


  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("ER"),
                                  2, JSD_SDO_DATA_DOUBLE,
                                  &config->epd.velocity_tracking_error)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("ER"),
                                  3, JSD_SDO_DATA_DOUBLE,
                                  &config->epd.position_tracking_error)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("PL"),
                                  2, JSD_SDO_DATA_FLOAT,
                                  &config->epd.peak_current_time)) {
    return 0;
  }

  // Note that the maximum current limit is also mapped to the RxPDO.
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("PL"),
                                  1, JSD_SDO_DATA_FLOAT,
                                  &config->epd.peak_current_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("CL"),
                                  1, JSD_SDO_DATA_FLOAT,
                                  &config->epd.continuous_current_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("CL"),
                                  2, JSD_SDO_DATA_FLOAT,
                                  &config->epd.motor_stuck_current_level_pct)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_lc_to_do("CL"), 3, JSD_SDO_DATA_FLOAT,
          &config->epd.motor_stuck_velocity_threshold)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("CL"),
                                  4, JSD_SDO_DATA_FLOAT,
                                  &config->epd.motor_stuck_timeout)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("HL"),
                                  2, JSD_SDO_DATA_DOUBLE,
                                  &config->epd.over_speed_threshold)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("LL"),
                                  3, JSD_SDO_DATA_DOUBLE,
                                  &config->epd.low_position_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("HL"),
                                  3, JSD_SDO_DATA_DOUBLE,
                                  &config->epd.high_position_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("BP"),
                                  1, JSD_SDO_DATA_I16,
                                  &config->epd.brake_engage_msec)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("BP"),
                                  2, JSD_SDO_DATA_I16,
                                  &config->epd.brake_disengage_msec)) {
    return 0;
  }

  int64_t ctrl_gs_mode_i64 = config->epd.ctrl_gain_scheduling_mode;
  if (ctrl_gs_mode_i64 != JSD_ELMO_GAIN_SCHEDULING_MODE_PRELOADED &&
      !jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("GS"),
                                  2, JSD_SDO_DATA_I64, &ctrl_gs_mode_i64)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("SF"),
                                  1, JSD_SDO_DATA_I64,
                                  &config->epd.smooth_factor)) {
    return 0;
  }

  // Verify startup parameters

  // Verify checksum from the drive matches checksum recorded in configuration
  uint64_t crc = 0;
  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("CZ"),
                                  1, JSD_SDO_DATA_U64, &crc)) {
    return 0;
  }
  MSG("EPD[%d] CRC = %lu", slave_id, crc);
  if (config->epd.crc == 0) {
    MSG("EPD[%d] drive parameter CRC check overridden", slave_id);
  } else {
    if (crc != config->epd.crc) {
      ERROR("EPD[%d] CRC mismatch - YAML value: %u, actual drive value: %lu",
            slave_id, config->epd.crc, crc);
      return 0;
    }
  }

  // Verify current limits
  float drive_max_current = 0;
  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("MC"),
                                  1, JSD_SDO_DATA_FLOAT, &drive_max_current)) {
    return 0;
  }
  MSG("EPD[%d] Drive Maximum Current is %f A", slave_id, drive_max_current);

  if (config->epd.peak_current_limit > drive_max_current) {
    // TODO(dloret): Check if the drive can even allow to set PL[1] if it is
    // greater than MC[1]. PL[1] is set above.
    ERROR("EPD[%d] Peak Current (%f) cannot exceed Drive Maximum Current (%f)",
          slave_id, config->epd.peak_current_limit, drive_max_current);
    return 0;
  }

  if (config->epd.continuous_current_limit > config->epd.peak_current_limit) {
    // TODO(dloret): this would actually disable CL[1] and is valid. Investigate
    // what is the implication of disabling CL[1].
    ERROR("EPD[%d] Continous Current (%f) should not exceed Peak Current (%f)",
          slave_id, config->epd.continuous_current_limit,
          config->epd.peak_current_limit);
    return 0;
  }

  // Display highest allowed control loop (UM[1]=1 -> current control loop,
  // UM[1]=2 -> velocity control loop, UM[1]=5 -> position control loop).
  int16_t um = 0;
  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id, jsd_epd_lc_to_do("UM"),
                                  1, JSD_SDO_DATA_I16, &um)) {
    return 0;
  }
  MSG("EPD[%d] UM[1] = %d", slave_id, um);

  return 1;
}

void jsd_epd_update_state_from_PDO_data(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;

  state->last_state_machine_state = state->pub.actual_state_machine_state;
  state->last_setpoint_ack        = state->setpoint_ack;

  state->pub.actual_position = state->txpdo.actual_position;
  state->pub.actual_velocity = state->txpdo.velocity_actual_value;
  state->pub.actual_current  = (double)state->txpdo.current_actual_value *
                              state->motor_rated_current / 1e6;

  state->pub.cmd_position = state->rxpdo.target_position;
  state->pub.cmd_velocity = state->rxpdo.target_velocity;
  state->pub.cmd_current =
      (double)state->rxpdo.target_torque * state->motor_rated_current / 1e6;
  state->pub.cmd_max_current =
      (double)state->rxpdo.max_current * state->motor_rated_current / 1e6;

  state->pub.cmd_ff_position = state->rxpdo.position_offset;
  state->pub.cmd_ff_velocity = state->rxpdo.velocity_offset;
  state->pub.cmd_ff_current =
      (double)state->rxpdo.torque_offset * state->motor_rated_current / 1e6;

  state->pub.cmd_prof_velocity     = state->rxpdo.profile_velocity;
  state->pub.cmd_prof_end_velocity = state->rxpdo.end_velocity;
  state->pub.cmd_prof_accel        = state->rxpdo.profile_accel;
  state->pub.cmd_prof_decel        = state->rxpdo.profile_decel;

  state->pub.actual_mode_of_operation = state->txpdo.mode_of_operation_display;
  // TODO(dloret): EGD code prints a change of mode of operation here.

  // Handle Statusword
  state->pub.actual_state_machine_state =
      state->txpdo.statusword & JSD_EPD_STATE_MACHINE_STATE_BITMASK;
  // TODO(dloret): EGD code prints a change of state here.
  if (state->pub.actual_state_machine_state !=
      state->last_state_machine_state) {
    MSG("EPD[%d] actual State Machine State changed to %s (0x%x)", slave_id,
        jsd_elmo_state_machine_state_to_string(
            state->pub.actual_state_machine_state),
        state->pub.actual_state_machine_state);

    state->prof_pos_waiting_setpoint_ack = false;

    if (state->pub.actual_state_machine_state ==
        JSD_ELMO_STATE_MACHINE_STATE_FAULT) {
      // TODO(dloret): Check if setting state->new_reset to false like in EGD
      // code is actually needed. Commands are handled after reading functions.
      state->fault_real_time = jsd_time_get_time_sec();
      state->fault_mono_time = jsd_time_get_mono_time_sec();

      jsd_sdo_signal_emcy_check(self);
    }
  }

  state->pub.warning        = state->txpdo.statusword >> 7 & 0x01;
  state->pub.target_reached = state->txpdo.statusword >> 10 & 0x01;
  state->setpoint_ack       = state->txpdo.statusword >> 12 & 0x01;
  state->pub.setpoint_ack_rise =
      (state->last_setpoint_ack == 0 && state->setpoint_ack == 1);

  // Handle Status Register
  state->pub.servo_enabled = state->txpdo.status_register_1 >> 4 & 0x01;
  state->fault_occured_when_enabled =
      state->txpdo.status_register_1 >> 6 & 0x01;
  // TODO(dloret): Double check this is a proper way to check STO status.
  state->pub.sto_engaged = !((state->txpdo.status_register_1 >> 25 & 0x01) &
                             (state->txpdo.status_register_1 >> 26 & 0x01));
  state->pub.motor_on    = state->txpdo.status_register_1 >> 22 & 0x01;
  state->pub.in_motion   = state->txpdo.status_register_1 >> 23 & 0x01;
  state->pub.hall_state  = state->txpdo.status_register_2 >> 0 & 0x07;

  // TODO(dloret): EGD code prints change in sto_engaged here.

  // Digital inputs
  state->interlock = state->txpdo.digital_inputs >> 3 & 0x01;
  for (int i = 0; i < JSD_EPD_NUM_DIGITAL_INPUTS; ++i) {
    state->pub.digital_inputs[i] =
        state->txpdo.digital_inputs >> (16 + i) & 0x01;
  }

  // Bus voltage
  state->pub.bus_voltage = state->txpdo.dc_link_circuit_voltage / 1000.0;

  // Analog input 1 voltage
  state->pub.analog_input_voltage = state->txpdo.analog_input_1 / 1000.0;

  // Analog input 2 analog to digital conversion
  state->pub.analog_input_adc = state->txpdo.analog_input_2;

  // Drive's temperature
  state->pub.drive_temperature = state->txpdo.drive_temperature_deg_c;
}

void jsd_epd_process_state_machine(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;

  switch (state->pub.actual_state_machine_state) {
    case JSD_ELMO_STATE_MACHINE_STATE_NOT_READY_TO_SWITCH_ON:
      // This case should never execute because it is an internal initial state
      // that cannot be monitored by the host.
      break;
    case JSD_ELMO_STATE_MACHINE_STATE_SWITCH_ON_DISABLED:
      // Transition to READY TO SWITCH ON
      state->rxpdo.controlword = JSD_EPD_STATE_MACHINE_CONTROLWORD_SHUTDOWN;
      break;
    case JSD_ELMO_STATE_MACHINE_STATE_READY_TO_SWITCH_ON:
      // Transition to SWITCHED ON
      state->rxpdo.controlword = JSD_EPD_STATE_MACHINE_CONTROLWORD_SWITCH_ON;
      break;
    case JSD_ELMO_STATE_MACHINE_STATE_SWITCHED_ON:
      // Startup, a fault, or the completion of a halt command (i.e. Quick Stop)
      // eventually land in this state. Transition to OPERATION ENABLED if a
      // reset command has been received.
      if (state->new_reset) {
        state->rxpdo.controlword =
            JSD_EPD_STATE_MACHINE_CONTROLWORD_ENABLE_OPERATION;
        state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_PROF_POS;
        state->rxpdo.mode_of_operation     = state->requested_mode_of_operation;
        state->new_reset                   = false;
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
        state->rxpdo.controlword = JSD_EPD_STATE_MACHINE_CONTROLWORD_QUICK_STOP;
        state->requested_mode_of_operation = JSD_EPD_MODE_OF_OPERATION_PROF_POS;
        state->rxpdo.mode_of_operation     = state->requested_mode_of_operation;
        break;
      }
      // Set the controlword to a known value before potentially setting its
      // mode of operation bits for profiled position mode. It does not
      // represent a transition.
      state->rxpdo.controlword =
          JSD_EPD_STATE_MACHINE_CONTROLWORD_ENABLE_OPERATION;
      jsd_epd_process_mode_of_operation(self, slave_id);
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
      while (num_error_pops < JSD_EPD_MAX_ERROR_POPS_PER_CYCLE &&
             jsd_error_cirq_pop(error_cirq, &error)) {
        if (ectime_to_sec(error.Time) > state->fault_real_time) {
          // Might want to handle other types of errors too in the future.
          if (error.Etype == EC_ERR_TYPE_EMERGENCY) {
            state->pub.emcy_error_code = error.ErrorCode;
            state->pub.fault_code = jsd_epd_get_fault_code_from_ec_error(error);

            // TODO(dloret): Remove printing to not affect real-time guarantees.
            ERROR("EPD[%d] EMCY code: 0x%X, fault description: %s", error.Slave,
                  state->pub.emcy_error_code,
                  jsd_epd_fault_code_to_string(state->pub.fault_code));

            // Transition to SWITCHED ON DISABLED
            state->rxpdo.controlword =
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
        WARNING("EPD[%d] in FAULT state but new EMCY code has not arrived",
                slave_id);

        state->pub.emcy_error_code = 0xFFFF;
        state->pub.fault_code      = JSD_EPD_FAULT_UNKNOWN;

        // Transition to SWITCHED ON DISABLED
        state->rxpdo.controlword =
            JSD_EPD_STATE_MACHINE_CONTROLWORD_FAULT_RESET;
      }
      break;
    default:
      ERROR(
          "EPD[%d] Unknown state machine state: 0x%x. This should never "
          "happen. Exiting.",
          slave_id, state->pub.actual_state_machine_state);
      assert(0);
  }
  state->new_motion_command = false;
  state->new_halt_command   = false;
}

void jsd_epd_process_mode_of_operation(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EPD_PRODUCT_CODE);

  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;

  // TODO(dloret): EGD code prints mode of operation change and warns about
  // changing mode of operation during motion.
  if (state->last_requested_mode_of_operation !=
      state->requested_mode_of_operation) {
    state->prof_pos_waiting_setpoint_ack = false;
  }
  state->last_requested_mode_of_operation = state->requested_mode_of_operation;

  switch (state->requested_mode_of_operation) {
    case JSD_EPD_MODE_OF_OPERATION_DISABLED:
      break;
    case JSD_EPD_MODE_OF_OPERATION_PROF_POS:
      jsd_epd_mode_of_op_handle_prof_pos(self, slave_id);
      break;
    case JSD_EPD_MODE_OF_OPERATION_PROF_VEL:
      jsd_epd_mode_of_op_handle_prof_vel(self, slave_id);
      break;
    case JSD_EPD_MODE_OF_OPERATION_PROF_TORQUE:
      jsd_epd_mode_of_op_handle_prof_torque(self, slave_id);
      break;
    case JSD_EPD_MODE_OF_OPERATION_CSP:
      jsd_epd_mode_of_op_handle_csp(self, slave_id);
      break;
    case JSD_EPD_MODE_OF_OPERATION_CSV:
      jsd_epd_mode_of_op_handle_csv(self, slave_id);
      break;
    case JSD_EPD_MODE_OF_OPERATION_CST:
      jsd_epd_mode_of_op_handle_cst(self, slave_id);
      break;
    default:
      ERROR(
          "EPD[%d] Mode of operation: 0x%x not implemented. This should never "
          "happen. Exiting.",
          slave_id, state->requested_mode_of_operation);
      assert(0);
  }
}

// TODO(dloret): Determine if it is appropriate to set to zero command variables
// of other motion modes when handling a particular mode. If the mode of
// operation does not transition in that cycle, the current motion could be
// abruptly interrupted.

void jsd_epd_mode_of_op_handle_csp(jsd_t* self, uint16_t slave_id) {
  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;
  jsd_epd_motion_command_t cmd   = state->motion_command;

  state->rxpdo.target_position = cmd.csp.target_position;
  state->rxpdo.position_offset = cmd.csp.position_offset;
  state->rxpdo.target_velocity = 0;
  state->rxpdo.velocity_offset = cmd.csp.velocity_offset;
  state->rxpdo.target_torque   = 0;
  state->rxpdo.torque_offset =
      cmd.csp.torque_offset_amps * 1e6 / state->motor_rated_current;
  state->rxpdo.profile_velocity = 0;
  state->rxpdo.end_velocity     = 0;
  state->rxpdo.profile_accel    = 0;
  state->rxpdo.profile_decel    = 0;

  state->rxpdo.mode_of_operation = JSD_EPD_MODE_OF_OPERATION_CSP;
}

void jsd_epd_mode_of_op_handle_csv(jsd_t* self, uint16_t slave_id) {
  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;
  jsd_epd_motion_command_t cmd   = state->motion_command;

  state->rxpdo.target_position = 0;
  state->rxpdo.position_offset = 0;
  state->rxpdo.target_velocity = cmd.csv.target_velocity;
  state->rxpdo.velocity_offset = cmd.csv.velocity_offset;
  state->rxpdo.target_torque   = 0;
  state->rxpdo.torque_offset =
      cmd.csv.torque_offset_amps * 1e6 / state->motor_rated_current;
  state->rxpdo.profile_velocity = 0;
  state->rxpdo.end_velocity     = 0;
  state->rxpdo.profile_accel    = 0;
  state->rxpdo.profile_decel    = 0;

  state->rxpdo.mode_of_operation = JSD_EPD_MODE_OF_OPERATION_CSV;
}

void jsd_epd_mode_of_op_handle_cst(jsd_t* self, uint16_t slave_id) {
  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;
  jsd_epd_motion_command_t cmd   = state->motion_command;

  state->rxpdo.target_position = 0;
  state->rxpdo.position_offset = 0;
  state->rxpdo.target_velocity = 0;
  state->rxpdo.velocity_offset = 0;
  state->rxpdo.target_torque =
      cmd.cst.target_torque_amps * 1e6 / state->motor_rated_current;
  state->rxpdo.torque_offset =
      cmd.cst.torque_offset_amps * 1e6 / state->motor_rated_current;
  state->rxpdo.profile_velocity = 0;
  state->rxpdo.end_velocity     = 0;
  state->rxpdo.profile_accel    = 0;
  state->rxpdo.profile_decel    = 0;

  state->rxpdo.mode_of_operation = JSD_EPD_MODE_OF_OPERATION_CST;
}

void jsd_epd_mode_of_op_handle_prof_pos(jsd_t* self, uint16_t slave_id) {
  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;
  jsd_epd_motion_command_t cmd   = state->motion_command;

  state->rxpdo.target_position  = cmd.prof_pos.target_position;
  state->rxpdo.position_offset  = 0;
  state->rxpdo.target_velocity  = 0;
  state->rxpdo.velocity_offset  = 0;
  state->rxpdo.target_torque    = 0;
  state->rxpdo.torque_offset    = 0;
  state->rxpdo.profile_velocity = cmd.prof_pos.profile_velocity;
  state->rxpdo.end_velocity     = cmd.prof_pos.end_velocity;
  state->rxpdo.profile_accel    = cmd.prof_pos.profile_accel;
  state->rxpdo.profile_decel    = cmd.prof_pos.profile_decel;

  // Having the new set-point bit on until the drive acknowledges reception of
  // the command is necessary so that the drive does not miss the bit when
  // changing between modes of operation.
  if (state->new_motion_command) {
    state->prof_pos_waiting_setpoint_ack = true;
  }
  if (state->prof_pos_waiting_setpoint_ack) {
    // Signal new set-point
    state->rxpdo.controlword |= (0x01 << 4);
  }
  if (state->pub.setpoint_ack_rise) {
    // After the rise of set-point acknowledge bit in statusword, new set-point
    // bit in controlword can be turned off.
    state->prof_pos_waiting_setpoint_ack = false;
  }

  // Request immediate change of set-point
  state->rxpdo.controlword |= (0x01 << 5);
  // Indicate whether motion is relative
  state->rxpdo.controlword |= (cmd.prof_pos.relative << 6);

  state->rxpdo.mode_of_operation = JSD_EPD_MODE_OF_OPERATION_PROF_POS;
}

void jsd_epd_mode_of_op_handle_prof_vel(jsd_t* self, uint16_t slave_id) {
  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;
  jsd_epd_motion_command_t cmd   = state->motion_command;

  state->rxpdo.target_position  = 0;
  state->rxpdo.position_offset  = 0;
  state->rxpdo.target_velocity  = cmd.prof_vel.target_velocity;
  state->rxpdo.velocity_offset  = 0;
  state->rxpdo.target_torque    = 0;
  state->rxpdo.torque_offset    = 0;
  state->rxpdo.profile_velocity = 0;
  state->rxpdo.end_velocity     = 0;
  state->rxpdo.profile_accel    = cmd.prof_vel.profile_accel;
  state->rxpdo.profile_decel    = cmd.prof_vel.profile_decel;

  state->rxpdo.mode_of_operation = JSD_EPD_MODE_OF_OPERATION_PROF_VEL;
}

void jsd_epd_mode_of_op_handle_prof_torque(jsd_t* self, uint16_t slave_id) {
  jsd_epd_private_state_t* state = &self->slave_states[slave_id].epd;
  jsd_epd_motion_command_t cmd   = state->motion_command;

  state->rxpdo.target_position = 0;
  state->rxpdo.position_offset = 0;
  state->rxpdo.target_velocity = 0;
  state->rxpdo.velocity_offset = 0;
  state->rxpdo.target_torque =
      cmd.prof_torque.target_torque_amps * 1e6 / state->motor_rated_current;
  state->rxpdo.torque_offset    = 0;
  state->rxpdo.profile_velocity = 0;
  state->rxpdo.end_velocity     = 0;
  state->rxpdo.profile_accel    = 0;
  state->rxpdo.profile_decel    = 0;

  state->rxpdo.mode_of_operation = JSD_EPD_MODE_OF_OPERATION_PROF_TORQUE;
}

jsd_epd_fault_code_t jsd_epd_get_fault_code_from_ec_error(ec_errort error) {
  jsd_epd_fault_code_t fault_code = JSD_EPD_FAULT_OKAY;
  switch (error.ErrorCode) {
    case 0x2340:
      fault_code = JSD_EPD_FAULT_SHORT_PROTECTION;
      break;
    case 0x3120:
      fault_code = JSD_EPD_FAULT_UNDER_VOLTAGE;
      break;
    case 0x3130:
      fault_code = JSD_EPD_FAULT_LOSS_OF_PHASE;
      break;
    case 0x3310:
      fault_code = JSD_EPD_FAULT_OVER_VOLTAGE;
      break;
    case 0x4210:
      fault_code = JSD_EPD_FAULT_MOTOR_OVER_TEMPERATURE;
      break;
    case 0x4310:
      fault_code = JSD_EPD_FAULT_DRIVE_OVER_TEMPERATURE;
      break;
    case 0x5280:
      fault_code = JSD_EPD_FAULT_GANTRY_YAW_ERROR_LIMIT_EXCEEDED;
      break;
    case 0x5441:
      fault_code = JSD_EPD_FAULT_EXTERNAL_INHIBIT_TRIGGERED;
      break;
    case 0x5442:
      fault_code = JSD_EPD_FAULT_ADDITIONAL_ABORT_ACTIVE;
      break;
    case 0x6181:
      fault_code = JSD_EPD_FAULT_VECTOR_ABORT;
      break;
    case 0x6300:
      fault_code = JSD_EPD_FAULT_RPDO_FAILED;
      break;
    case 0x7121:
      fault_code = JSD_EPD_FAULT_MOTOR_STUCK;
      break;
    case 0x7300:
      fault_code = JSD_EPD_FAULT_FEEDBACK_ERROR;
      break;
    case 0x7380:
      fault_code = JSD_EPD_FAULT_HALL_MAIN_FEEDBACK_MISMATCH;
      break;
    case 0x7381:
      fault_code = JSD_EPD_FAULT_HALL_BAD_CHANGE;
      break;
    case 0x7382:
      fault_code = JSD_EPD_FAULT_COMMUTATION_PROCESS_FAIL;
      break;
    case 0x8110:
      fault_code = JSD_EPD_FAULT_CAN_MESSAGE_LOST;
      break;
    case 0x8130:
      fault_code = JSD_EPD_FAULT_SYNC_OR_FRAME_LOSS;
      break;
    case 0x8140:
      fault_code = JSD_EPD_FAULT_RECOVERED_FROM_BUS_OFF;
      break;
    case 0x8200:
      fault_code = JSD_EPD_FAULT_ACCESS_NON_CONFIGURED_RPDO;
      break;
    case 0x8210:
      fault_code = JSD_EPD_FAULT_INCORRECT_RPDO_LENGTH;
      break;
    case 0x8311:
      fault_code = JSD_EPD_FAULT_PEAK_CURRENT_EXCEEDED;
      break;
    case 0x8480:
      fault_code = JSD_EPD_FAULT_SPEED_TRACKING_ERROR;
      break;
    case 0x8481:
      fault_code = JSD_EPD_FAULT_SPEED_LIMIT_EXCEEDED;
      break;
    case 0x8611:
      fault_code = JSD_EPD_FAULT_POSITION_TRACKING_ERROR;
      break;
    case 0x8680:
      fault_code = JSD_EPD_FAULT_POSITION_LIMIT_EXCEEDED;
      break;
    case 0xFF02:
      fault_code = JSD_EPD_FAULT_CAN_INTERPOLATED_MODE_EMERGENCY;
      break;
    case 0xFF10:
      fault_code = JSD_EPD_FAULT_CANNOT_START_MOTOR;
      break;
    case 0xFF20:
      fault_code = JSD_EPD_FAULT_STO_ENGAGED;
      break;
    case 0xFF30:
      fault_code = JSD_EPD_FAULT_MOTOR_DISABLE_COMMAND;
      break;
    case 0xFF34:
      fault_code = JSD_EPD_FAULT_KINEMATICS_ERROR;
      break;
    case 0xFF35:
      fault_code = JSD_EPD_FAULT_GANTRY_MASTER_ERROR;
      break;
    case 0xFF40:
      fault_code = JSD_EPD_FAULT_GANTRY_SLAVE_DISABLED;
      break;
    case 0xFF50:
      fault_code = JSD_EPD_FAULT_GANTRY_ATTACHED_SLAVE_FAULT;
      break;
    default:
      fault_code = JSD_EPD_FAULT_UNKNOWN;
  }
  return fault_code;
}
