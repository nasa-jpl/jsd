#include "jsd/jsd_egd.h"

#include <assert.h>
#include <float.h>
#include <string.h>

#include "ethercat.h"
#include "jsd/jsd.h"
#include "jsd/jsd_sdo.h"

static void set_controlword(jsd_t* self, uint16_t slave_id,
                            uint16_t controlword) {
  jsd_egd_private_state_t* state = &self->slave_states[slave_id].egd;

  /*
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_CS) {
    state->rxpdo_cs.controlword = controlword;
  } else if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    state->rxpdo_prof.controlword = controlword;
  } else {
    ERROR("Bad Drive Cmd mode: %d", config->egd.drive_cmd_mode);
  }
  */
  state->rxpdo_cs.controlword   = controlword;
  state->rxpdo_prof.controlword = controlword;
}

static uint16_t get_controlword(jsd_t* self, uint16_t slave_id) {
  jsd_egd_private_state_t* state  = &self->slave_states[slave_id].egd;
  jsd_slave_config_t*      config = &self->slave_configs[slave_id];

  uint16_t cw = 0;

  if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_CS) {
    cw = state->rxpdo_cs.controlword;
  } else if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    cw = state->rxpdo_prof.controlword;
  } else {
    ERROR("Bad Drive Cmd mode: %d", config->egd.drive_cmd_mode);
  }
  return cw;
}

static void set_mode_of_operation(jsd_t* self, uint16_t slave_id,
                                  int8_t mode_of_operation) {
  jsd_egd_private_state_t* state = &self->slave_states[slave_id].egd;

  /*
  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_CS) {
    state->rxpdo_cs.mode_of_operation = mode_of_operation;
  } else if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    state->rxpdo_prof.mode_of_operation = mode_of_operation;
  } else {
    ERROR("Bad Drive Cmd mode: %d", config->egd.drive_cmd_mode);
  }
  */
  state->rxpdo_cs.mode_of_operation   = mode_of_operation;
  state->rxpdo_prof.mode_of_operation = mode_of_operation;
}
/*
static int8_t get_mode_of_operation(jsd_t* self, uint16_t slave_id){
  jsd_egd_private_state_t* state = &self->slave_states[slave_id].egd;
  jsd_slave_config_t* config = &self->slave_configs[slave_id];

  int8_t mop = 0;

  if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_CS) {
    mop = state->rxpdo_cs.mode_of_operation;
  } else if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    mop = state->rxpdo_prof.mode_of_operation;
  } else {
    ERROR("Bad Drive Cmd mode: %d", config->egd.drive_cmd_mode);
  }
  return mop;

}
*/

/****************************************************
 * Public functions
 ****************************************************/

const jsd_egd_state_t* jsd_egd_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);
  return &self->slave_states[slave_id].egd.pub;
}

void jsd_egd_reset(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);
  double now = jsd_get_time_sec();
  if ((now - self->slave_states[slave_id].egd.last_reset_time) >
      JSD_EGD_RESET_DERATE_SEC) {
    self->slave_states[slave_id].egd.new_reset       = true;
    self->slave_states[slave_id].egd.last_reset_time = now;
  } else {
    WARNING(
        "EGD Reset Derate Protection feature is preventing reset, ignoring "
        "request");
  }
}

void jsd_egd_halt(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);
  self->slave_states[slave_id].egd.new_halt_command = true;
}

// A halt achieves the same effect, wanted this function for public API
void jsd_egd_disable_drive(jsd_t* self, uint16_t slave_id) {
  jsd_egd_halt(self, slave_id);
}

void jsd_egd_set_digital_output(jsd_t* self, uint16_t slave_id,
                                uint8_t digital_output_index,
                                uint8_t output_level) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);
  assert(digital_output_index < JSD_EGD_NUM_DIGITAL_OUTPUTS);

  if (self->slave_configs[slave_id].egd.drive_cmd_mode !=
      JSD_EGD_DRIVE_CMD_MODE_CS) {
    ERROR("Drive cmd mode: %d not suitable for digital output ",
          self->slave_configs[slave_id].egd.drive_cmd_mode);
    return;
  }

  jsd_egd_private_state_t* state = &self->slave_states[slave_id].egd;
  if (output_level > 0) {
    state->rxpdo_cs.digital_outputs |= (0x01 << (16 + digital_output_index));
  } else {
    state->rxpdo_cs.digital_outputs &= ~(0x01 << (16 + digital_output_index));
  }
}

void jsd_egd_set_peak_current(jsd_t* self, uint16_t slave_id,
                              double peak_current) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  jsd_egd_private_state_t* state  = &self->slave_states[slave_id].egd;
  jsd_slave_config_t*      config = &self->slave_configs[slave_id];

  uint16_t current = peak_current * 1e6 / state->motor_rated_current;

  if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_CS) {
    state->rxpdo_cs.max_current = current;

  } else if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    state->rxpdo_prof.max_current = current;

  } else {
    ERROR("Bad Drive Cmd mode: %d", config->egd.drive_cmd_mode);
  }
}

void jsd_egd_set_motion_command_prof_pos(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_prof_pos_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  if (self->slave_configs[slave_id].egd.drive_cmd_mode !=
      JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    ERROR("Drive cmd mode: %d not suitable for profiled commands",
          self->slave_configs[slave_id].egd.drive_cmd_mode);
    return;
  }

  jsd_egd_private_state_t* state     = &self->slave_states[slave_id].egd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_PROF_POS;
  state->motion_command.prof_pos     = motion_command;
}

void jsd_egd_set_motion_command_prof_vel(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_prof_vel_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  if (self->slave_configs[slave_id].egd.drive_cmd_mode !=
      JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    ERROR("Drive cmd mode: %d not suitable for profiled commands",
          self->slave_configs[slave_id].egd.drive_cmd_mode);
    return;
  }

  jsd_egd_private_state_t* state     = &self->slave_states[slave_id].egd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_PROF_VEL;
  state->motion_command.prof_vel     = motion_command;
}

void jsd_egd_set_motion_command_prof_torque(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_prof_torque_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  if (self->slave_configs[slave_id].egd.drive_cmd_mode !=
      JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    ERROR("Drive cmd mode: %d not suitable for profiled commands",
          self->slave_configs[slave_id].egd.drive_cmd_mode);
    return;
  }

  jsd_egd_private_state_t* state     = &self->slave_states[slave_id].egd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_PROF_TORQUE;
  state->motion_command.prof_torque  = motion_command;
}

void jsd_egd_set_motion_command_csp(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_csp_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  if (self->slave_configs[slave_id].egd.drive_cmd_mode !=
      JSD_EGD_DRIVE_CMD_MODE_CS) {
    ERROR("Drive cmd mode: %d not suitable for Cyc. Sync. commands",
          self->slave_configs[slave_id].egd.drive_cmd_mode);
    return;
  }

  jsd_egd_private_state_t* state     = &self->slave_states[slave_id].egd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_CSP;
  state->motion_command.csp          = motion_command;
}

void jsd_egd_set_motion_command_csv(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_csv_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  if (self->slave_configs[slave_id].egd.drive_cmd_mode !=
      JSD_EGD_DRIVE_CMD_MODE_CS) {
    ERROR("Drive cmd mode: %d not suitable for Cyc. Sync. commands",
          self->slave_configs[slave_id].egd.drive_cmd_mode);
    return;
  }

  jsd_egd_private_state_t* state     = &self->slave_states[slave_id].egd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_CSV;
  state->motion_command.csv          = motion_command;
}

void jsd_egd_set_motion_command_cst(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_cst_t motion_command) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  if (self->slave_configs[slave_id].egd.drive_cmd_mode !=
      JSD_EGD_DRIVE_CMD_MODE_CS) {
    ERROR("Drive cmd mode: %d not suitable for Cyc. Sync. commands",
          self->slave_configs[slave_id].egd.drive_cmd_mode);
    return;
  }

  jsd_egd_private_state_t* state     = &self->slave_states[slave_id].egd;
  state->new_motion_command          = true;
  state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_CST;
  state->motion_command.cst          = motion_command;
}

char* jsd_egd_mode_of_operation_to_string(jsd_egd_mode_of_operation_t mode) {
  switch (mode) {
    case JSD_EGD_MODE_OF_OPERATION_DISABLED:
      return "Disabled";
      break;
    case JSD_EGD_MODE_OF_OPERATION_PROF_POS:
      return "Profiled Position";
      break;
    case JSD_EGD_MODE_OF_OPERATION_PROF_VEL:
      return "Profiled Velocity";
      break;
    case JSD_EGD_MODE_OF_OPERATION_PROF_TORQUE:
      return "Profiled Torque";
      break;
    case JSD_EGD_MODE_OF_OPERATION_CSP:
      return "Cyclic Synchronous Position";
      break;
    case JSD_EGD_MODE_OF_OPERATION_CSV:
      return "Cyclic Synchronous Velocity";
      break;
    case JSD_EGD_MODE_OF_OPERATION_CST:
      return "Cyclic Synchronous Torque";
      break;
    default: {
      char str[JSD_NAME_LEN];
      snprintf(str, JSD_NAME_LEN, "Bad Mode of Operation: 0x%x", mode);
      return strdup(str);
      break;
    }
  }
}

char* jsd_egd_state_machine_state_to_string(
    jsd_egd_state_machine_state_t state) {
  switch (state) {
    case JSD_EGD_STATE_MACHINE_STATE_NOT_READY_TO_SWITCH_ON:
      return "Not Ready to Switch On";
      break;
    case JSD_EGD_STATE_MACHINE_STATE_SWITCH_ON_DISABLED:
      return "Switch On Disabled";
      break;
    case JSD_EGD_STATE_MACHINE_STATE_READY_TO_SWITCH_ON:
      return "Ready to Switch On";
      break;
    case JSD_EGD_STATE_MACHINE_STATE_SWITCHED_ON:
      return "Switched On";
      break;
    case JSD_EGD_STATE_MACHINE_STATE_OPERATION_ENABLED:
      return "Operation Enabled";
      break;
    case JSD_EGD_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE:
      return "Quick Stop Active";
      break;
    case JSD_EGD_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE:
      return "Fault Reaction Active";
      break;
    case JSD_EGD_STATE_MACHINE_STATE_FAULT:
      return "Fault";
      break;
    default: {
      char str[JSD_NAME_LEN];
      snprintf(str, JSD_NAME_LEN, "Bad State Machine State: 0x%x", state);
      return strdup(str);
      break;
    }
  }
}

char* jsd_egd_fault_code_to_string(jsd_egd_fault_code_t fault_code) {
  switch (fault_code) {
    case JSD_EGD_FAULT_OKAY:
      return "JSD_EGD_FAULT_OKAY";
      break;
    case JSD_EGD_FAULT_RESERVED:
      return "JSD_EGD_FAULT_RESERVED";
      break;
    case JSD_EGD_FAULT_OVER_CURRENT:
      return "JSD_EGD_FAULT_OVER_CURRENT";
      break;
    case JSD_EGD_FAULT_SHORT_CIRCUIT:
      return "JSD_EGD_FAULT_SHORT_CIRCUIT";
      break;
    case JSD_EGD_FAULT_UNDER_VOLTAGE:
      return "JSD_EGD_FAULT_UNDER_VOLTAGE";
      break;
    case JSD_EGD_FAULT_LOSS_OF_PHASE:
      return "JSD_EGD_FAULT_LOSS_OF_PHASE";
      break;
    case JSD_EGD_FAULT_OVER_VOLTAGE:
      return "JSD_EGD_FAULT_OVER_VOLTAGE";
      break;
    case JSD_EGD_FAULT_DRIVE_OVER_TEMP:
      return "JSD_EGD_FAULT_DRIVE_OVER_TEMP";
      break;
    case JSD_EGD_FAULT_ECAM_DIFF:
      return "JSD_EGD_FAULT_ECAM_DIFF";
      break;
    case JSD_EGD_FAULT_TIMING_ERROR:
      return "JSD_EGD_FAULT_TIMING_ERROR";
      break;
    case JSD_EGD_FAULT_MOTOR_DISABLED_BY_SWITCH:
      return "JSD_EGD_FAULT_MOTOR_DISABLED_BY_SWITCH";
      break;

    case JSD_EGD_FAULT_ABORT_MOTION:
      return "JSD_EGD_FAULT_ABORT_MOTION";
      break;
    case JSD_EGD_FAULT_CPU_STACK_OVERFLOW:
      return "JSD_EGD_FAULT_CPU_STACK_OVERFLOW";
      break;
    case JSD_EGD_FAULT_CPU_FATAL_EXCEPTION:
      return "JSD_EGD_FAULT_CPU_FATAL_EXCEPTION";
      break;
    case JSD_EGD_FAULT_USER_PROG_ABORTED:
      return "JSD_EGD_FAULT_USER_PROG_ABORTED";
      break;
    case JSD_EGD_FAULT_RPDO_MAP_ERROR:
      return "JSD_EGD_FAULT_RPDO_MAP_ERROR";
      break;
    case JSD_EGD_FAULT_INCONSISTENT_DATABASE:
      return "JSD_EGD_FAULT_INCONSISTENT_DATABASE";
      break;
    case JSD_EGD_FAULT_MOTOR_STUCK:
      return "JSD_EGD_FAULT_MOTOR_STUCK";
      break;
    case JSD_EGD_FAULT_FEEDBACK_ERROR:
      return "JSD_EGD_FAULT_FEEDBACK_ERROR";
      break;
    case JSD_EGD_FAULT_COMMUTATION_FAILED:
      return "JSD_EGD_FAULT_COMMUTATION_FAILED";
      break;
    case JSD_EGD_FAULT_FEEBACK_LOSS:
      return "JSD_EGD_FAULT_FEEBACK_LOSS";
      break;
    case JSD_EGD_FAULT_DIGITAL_HALL_BAD_CHANGE:
      return "JSD_EGD_FAULT_DIGITAL_HALL_BAD_CHANGE";
      break;
    case JSD_EGD_FAULT_COMMUTATION_PROCESS_FAIL:
      return "JSD_EGD_FAULT_COMMUTATION_PROCESS_FAIL";
      break;

    case JSD_EGD_FAULT_CAN_MSG_LOST:
      return "JSD_EGD_FAULT_CAN_MSG_LOST";
      break;
    case JSD_EGD_FAULT_HEARTBEAT_EVENT:
      return "JSD_EGD_FAULT_HEARTBEAT_EVENT";
      break;
    case JSD_EGD_FAULT_RECOVER_BUS_OFF:
      return "JSD_EGD_FAULT_RECOVER_BUS_OFF";
      break;
    case JSD_EGD_FAULT_NMT_PROTOCOL_ERROR:
      return "JSD_EGD_FAULT_NMT_PROTOCOL_ERROR";
      break;
    case JSD_EGD_FAULT_ACCESS_UNCONFIGURED_RPDO:
      return "JSD_EGD_FAULT_ACCESS_UNCONFIGURED_RPDO";
      break;
    case JSD_EGD_FAULT_PEAK_CURRENT_EXCEEDED:
      return "JSD_EGD_FAULT_PEAK_CURRENT_EXCEEDED";
      break;
    case JSD_EGD_FAULT_FAILED_ELECTRICAL_ZERO:
      return "JSD_EGD_FAULT_FAILED_ELECTRICAL_ZERO";
      break;
    case JSD_EGD_FAULT_CANNOT_TUNE:
      return "JSD_EGD_FAULT_CANNOT_TUNE";
      break;
    case JSD_EGD_FAULT_SPEED_TRACKING_ERROR:
      return "JSD_EGD_FAULT_SPEED_TRACKING_ERROR";
      break;
    case JSD_EGD_FAULT_SPEED_LIMIT_EXCEEDED:
      return "JSD_EGD_FAULT_SPEED_LIMIT_EXCEEDED";
      break;

    case JSD_EGD_FAULT_POSITION_TRACKING_ERROR:
      return "JSD_EGD_FAULT_POSITION_TRACKING_ERROR";
      break;
    case JSD_EGD_FAULT_POSITION_LIMIT_EXCEEDED:
      return "JSD_EGD_FAULT_POSITION_LIMIT_EXCEEDED";
      break;
    case JSD_EGD_FAULT_BAD_DATA_0XFF00:
      return "JSD_EGD_FAULT_BAD_DATA_0XFF00";
      break;

    case JSD_EGD_FAULT_USER_PROG_EMIT:
      return "JSD_EGD_FAULT_USER_PROG_EMIT";
      break;
    case JSD_EGD_FAULT_BAD_DATA_0XFF02:
      return "JSD_EGD_FAULT_BAD_DATA_0XFF02";
      break;
    case JSD_EGD_FAULT_CANNOT_START_MOTOR:
      return "JSD_EGD_FAULT_CANNOT_START_MOTOR";
      break;
    case JSD_EGD_FAULT_STO_ENGAGED:
      return "JSD_EGD_FAULT_STO_ENGAGED";
      break;
    case JSD_EGD_FAULT_MODULO_OVERFLOW:
      return "JSD_EGD_FAULT_MODULO_OVERFLOW";
      break;
    case JSD_EGD_FAULT_NUMERIC_OVERFLOW:
      return "JSD_EGD_FAULT_NUMERIC_OVERFLOW";
      break;
    case JSD_EGD_FAULT_GANTRY_SLAVE_DISABLED:
      return "JSD_EGD_FAULT_GANTRY_SLAVE_DISABLED";
      break;
    case JSD_EGD_FAULT_SDO_ERROR:
      return "JSD_EGD_FAULT_SDO_ERROR";
      break;
    case JSD_EGD_FAULT_UNKNOWN:
      return "JSD_EGD_FAULT_UNKNOWN";
      break;

    default: {
      char str[JSD_NAME_LEN];
      snprintf(str, JSD_NAME_LEN, "Bad fault code: 0x%x", fault_code);
      return strdup(str);
      break;
    }
  }
}

void jsd_egd_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  jsd_egd_read_PDO_data(self, slave_id);
  jsd_egd_update_state_from_PDO_data(self, slave_id);
  jsd_egd_async_sdo_process(self, slave_id);
}

void jsd_egd_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  jsd_egd_process_state_machine(self, slave_id);
  jsd_egd_write_PDO_data(self, slave_id);
}

///////////////////  ASYNC SDO /////////////////////////////
//
uint16_t jsd_egd_tlc_to_do(char tlc[2]) {
  if ((tlc[0] < 65 || tlc[0] > 90) || (tlc[1] < 65 || tlc[1] > 90)) {
    ERROR("Two-Letter Command string must be uppercased: %s", tlc);
  }
  // idea: if lowercase attempt to promote to uppercase

  uint16_t index = 0x3000 + (26 * (tlc[0] - 65)) + (tlc[1] - 65);

  if (index < 0x3000 || index > 0x3FFF) {
    ERROR(
        "Two-Letter Command converstion is out of range: %s -> 0x%X not in "
        "(0x3000,0x3FFF)",
        tlc, index);
  }

  MSG_DEBUG("Converted TLC: %s to index 0x%X", tlc, index);

  return index;
}

void jsd_egd_async_sdo_set_drive_position(jsd_t* self, uint16_t slave_id,
                                          int32_t position) {
  jsd_sdo_set_param_async(self, slave_id, jsd_egd_tlc_to_do("PX"), 1,
                          JSD_SDO_DATA_I32, &position);
}

void jsd_egd_async_sdo_set_unit_mode(jsd_t* self, uint16_t slave_id,
                                     int32_t mode) {
  jsd_sdo_set_param_async(self, slave_id, jsd_egd_tlc_to_do("UM"), 1,
                          JSD_SDO_DATA_I32, &mode);
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_egd_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man == JSD_ELMO_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  config->PO2SO_success      = false;  // only set true in PO2SO callback

  // Disables Complete Access (CA) in EGD devices
  // This was needed to make PDO mapping work TODO revisit why
  slave->CoEdetails &= ~ECT_COEDET_SDOCA;

  slave->PO2SOconfigx = jsd_egd_PO2SO_config;

  jsd_egd_private_state_t* state = &self->slave_states[slave_id].egd;
  state->last_reset_time         = 0;

  if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_CS) {
    MSG_DEBUG("rxpdo_cs size: %zu Bytes", sizeof(jsd_egd_rxpdo_data_cs_mode_t));
  } else if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    MSG_DEBUG("rxpdo_prof size: %zu Bytes",
              sizeof(jsd_egd_rxpdo_data_profiled_mode_t));
  } else {
    ERROR("Bad Drive Cmd mode: %d", config->egd.drive_cmd_mode);
  }

  // this has to be performed up front to share with device_state
  state->motor_rated_current = config->egd.continuous_current_limit * 1000;

  if (state->motor_rated_current == 0) {
    ERROR("continuous_current_limit not set on EGD[%d]", slave_id);
    return false;
  }
  jsd_egd_set_peak_current(self, slave_id, config->egd.peak_current_limit);

  return true;
}

int jsd_egd_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within th ecx_context and extract it here
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;
  jsd_slave_config_t* config = &slave_configs[slave_id];

  if (!jsd_egd_config_PDO_mapping(ecx_context, slave_id, config)) {
    ERROR("Failed to map PDO parameters on EGD slave %u", slave_id);
    return 0;
  }

  if (!jsd_egd_config_COE_params(ecx_context, slave_id, config)) {
    ERROR("Failed to set COE parameters on EGD slave %u", slave_id);
    return 0;
  }

  if (!jsd_egd_config_TLC_params(ecx_context, slave_id, config)) {
    ERROR("Failed to set TLC parameters on EGD slave %u", slave_id);
    return 0;
  }

  config->PO2SO_success = true;
  SUCCESS("EGD[%d] drive parameters successfully configured and verified",
          slave_id);
  return 1;
}

int jsd_egd_config_PDO_mapping(ecx_contextt* ecx_context, uint16_t slave_id,
                               jsd_slave_config_t* config) {
  MSG_DEBUG("Attempting to map custom EGD PDOs...");

  //////////////// RxPDO Mapping //////////////////////////
  if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_CS) {
    uint16_t map_output_pdos_1607[] = {
        0x0007,          // num mapped params
        0x0020, 0x60FF,  // target_velocity
        0x0010, 0x6071,  // target_torque
        0x0020, 0x60B0,  // position_offset
        0x0020, 0x60B1,  // velocity_offset
        0x0010, 0x60B2,  // torque_offset
        0x0008, 0x6060,  // mode_of_operation
        0x0010, 0x6073,  // max_current
    };

    if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1607, 0x00,
                                       sizeof(map_output_pdos_1607),
                                       &map_output_pdos_1607)) {
      return 0;
    }

    uint16_t map_output_RxPDO[] = {0x0002, 0x1600, 0x1607};
    if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1C12, 0x00,
                                       sizeof(map_output_RxPDO),
                                       &map_output_RxPDO)) {
      return 0;
    }

  } else if (config->egd.drive_cmd_mode == JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    uint16_t map_output_pdos_1607[] = {
        0x0006,          // num mapped params
        0x0010, 0x6071,  // target_torque
        0x0020, 0x6081,  // profile_velocity
        0x0020, 0x6083,  // profile_accel
        0x0020, 0x6084,  // profile_decel
        0x0020, 0x6082,  // end_velocity
        0x0008, 0x6060,  // mode_of_operation
    };

    if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1607, 0x00,
                                       sizeof(map_output_pdos_1607),
                                       &map_output_pdos_1607)) {
      return 0;
    }

    uint16_t map_output_RxPDO[] = {0x0002, 0x1604, 0x1607};
    if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1C12, 0x00,
                                       sizeof(map_output_RxPDO),
                                       &map_output_RxPDO)) {
      return 0;
    }

  } else {
    ERROR("Unknown Drive Command Mode! %d", config->egd.drive_cmd_mode);
    return 0;
  }

  //////////////// TxPDO Mapping //////////////////////////

  uint16_t map_input_pdos_1a07[] = {
      0x0007,          // num mapped params
      0x0110, 0x2205,  // Analog Input
      0x0020, 0x6069,  // Velocity Actual Value
      0x0010, 0x6078,  // Current Actual Value
      0x0020, 0x1002,  // Status Register
      0x0008, 0x6061,  // Mode of Operation Display
      0x0020, 0x6079,  // DC link circuit voltage
      0x0020, 0x2203,  // Application object (configed for drive temp)
  };

  if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1A07, 0x00,
                                     sizeof(map_input_pdos_1a07),
                                     &map_input_pdos_1a07)) {
    return 0;
  }

  uint16_t map_input_TxPDO[] = {0x0002, 0x1A00, 0x1A07};

  if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1C13, 0x00,
                                     sizeof(map_input_TxPDO),
                                     &map_input_TxPDO)) {
    return 0;
  }

  return 1;
}

int jsd_egd_config_COE_params(ecx_contextt* ecx_context, uint16_t slave_id,
                              jsd_slave_config_t* config) {
  // set the Application object (0x2203) to report drive temp
  uint32_t app_obj_config_word = 0x01 << (16 + 3);
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x2F41, 0x00,
                                  JSD_SDO_DATA_U32, &app_obj_config_word)) {
    return 0;
  }

  // by default, put drive in PROF_POS mode
  int8_t ctrl_word = JSD_EGD_MODE_OF_OPERATION_PROF_POS;

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x6060, 0x00,
                                  JSD_SDO_DATA_I8, &ctrl_word)) {
    return 0;
  }

  // Set relative motion to be relative of actual position
  uint16_t pos_optcode = 0x02;

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x60F2, 0x00,
                                  JSD_SDO_DATA_U16, &pos_optcode)) {
    return 0;
  }

  // Set interpolation time period
  // This drive setting can be configured to microsec
  // Contact the current JSD maintainer to add
  int8_t loop_period_ms = config->egd.loop_period_ms;
  if (loop_period_ms < 1) {
    loop_period_ms = 1;
  }
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x60C2, 1,
                                  JSD_SDO_DATA_I8, (void*)&loop_period_ms)) {
    return 0;
  }

  // set Extrapolation cycles timeout (5 cycles based on ECAT lib testing)
  int16_t extra_cycles = 5;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x2F75, 0,
                                  JSD_SDO_DATA_I16, (void*)&extra_cycles)) {
    return 0;
  }

  // Set quick-stop function
  int16_t quick_stop_option = 2;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x605A, 0,
                                  JSD_SDO_DATA_I16,
                                  (void*)&quick_stop_option)) {
    return 0;
  }

  //// set motor rated current CL[1]
  uint32_t motor_rated_current = config->egd.continuous_current_limit * 1000.0;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x6075, 0,
                                  JSD_SDO_DATA_U32,
                                  (void*)&motor_rated_current)) {
    return 0;
  }

  //// set torque slope for profiled torque commands
  uint32_t torque_slope = config->egd.torque_slope * 1e6 / motor_rated_current;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x6087, 0,
                                  JSD_SDO_DATA_U32, (void*)&torque_slope)) {
    return 0;
  }

  // set max motor speed
  //   0x6080 accepts units of RPM and internally converts to 
  //   counts per second according to CA[18]. Every other speed in in cnts/sec
  //   so to keep the JSD api consistent perform the adjustment here.
  uint32_t ca_18;
  if (!jsd_sdo_get_param_blocking(
          ecx_context, slave_id, jsd_egd_tlc_to_do("CA"), 18, JSD_SDO_DATA_U32,
          (void*)&ca_18)) {
    return 0;
  }
  MSG("EGD[%d] read CA[18] = %u cnts per rev", slave_id, ca_18);

  double max_motor_speed_rpm = (config->egd.max_motor_speed) / (double)ca_18 * 60.0;

  int32_t max_motor_speed_rpm_int = (int32_t)max_motor_speed_rpm;

  MSG("EGD[%d] max_motor_speed_rpm = %lf as int: %i", slave_id, 
      max_motor_speed_rpm, 
      max_motor_speed_rpm_int);

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x6080, 0,
                                  JSD_SDO_DATA_I32,
                                  (void*)&max_motor_speed_rpm_int)) {
    return 0;
  }

  return 1;
}

int jsd_egd_config_TLC_params(ecx_contextt* ecx_context, uint16_t slave_id,
                              jsd_slave_config_t* config) {
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id,
                                  jsd_egd_tlc_to_do("AC"), 1, JSD_SDO_DATA_U32,
                                  (void*)&config->egd.max_profile_accel)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id,
                                  jsd_egd_tlc_to_do("DC"), 1, JSD_SDO_DATA_U32,
                                  (void*)&config->egd.max_profile_decel)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_egd_tlc_to_do("ER"), 2, JSD_SDO_DATA_I32,
          (void*)&config->egd.velocity_tracking_error)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_egd_tlc_to_do("ER"), 3, JSD_SDO_DATA_I32,
          (void*)&config->egd.position_tracking_error)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_egd_tlc_to_do("PL"), 2, JSD_SDO_DATA_FLOAT,
          (void*)&config->egd.peak_current_time)) {
    return 0;
  }

  // PL[1] does not need to be set here since it is updated synchronously
  // with every PDO exchange

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_egd_tlc_to_do("CL"), 1, JSD_SDO_DATA_FLOAT,
          (void*)&config->egd.continuous_current_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_egd_tlc_to_do("CL"), 2, JSD_SDO_DATA_FLOAT,
          (void*)&config->egd.motor_stuck_current_level_pct)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_egd_tlc_to_do("CL"), 3, JSD_SDO_DATA_FLOAT,
          (void*)&config->egd.motor_stuck_velocity_threshold)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_egd_tlc_to_do("CL"), 4, JSD_SDO_DATA_FLOAT,
          (void*)&config->egd.motor_stuck_timeout)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id,
                                  jsd_egd_tlc_to_do("HL"), 2, JSD_SDO_DATA_I32,
                                  (void*)&config->egd.over_speed_threshold)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id,
                                  jsd_egd_tlc_to_do("LL"), 3, JSD_SDO_DATA_I32,
                                  (void*)&config->egd.low_position_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id,
                                  jsd_egd_tlc_to_do("HL"), 3, JSD_SDO_DATA_I32,
                                  (void*)&config->egd.high_position_limit)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id,
                                  jsd_egd_tlc_to_do("BP"), 1, JSD_SDO_DATA_I32,
                                  (void*)&config->egd.brake_engage_msec)) {
    return 0;
  }

  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id,
                                  jsd_egd_tlc_to_do("BP"), 2, JSD_SDO_DATA_I32,
                                  (void*)&config->egd.brake_disengage_msec)) {
    return 0;
  }

  // set smooth factor
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id,
                                  jsd_egd_tlc_to_do("SF"), 1, JSD_SDO_DATA_I32,
                                  (void*)&config->egd.smooth_factor)) {
    return 0;
  }

  /////// Verify startup parameters ////////////////

  int32_t crc = 0;
  if (!jsd_sdo_get_param_blocking(ecx_context, slave_id,
                                  jsd_egd_tlc_to_do("OV"), 52, JSD_SDO_DATA_I32,
                                  (void*)&crc)) {
    return 0;
  }
  MSG("EGD[%d] CRC = %d", slave_id, crc);

  if (config->egd.crc == INT32_MIN) {
    MSG("EGD[%d] Drive parameter CRC check overridden", slave_id);

  } else {
    if (crc != config->egd.crc) {
      ERROR("EGD[%d] CRC mismatch - YAML value: %i  actual drive value: %i",
            slave_id, config->egd.crc, crc);
      return 0;
    }
  }

  float drive_max_current = 0;
  if (!jsd_sdo_get_param_blocking(
          ecx_context, slave_id, jsd_egd_tlc_to_do("MC"), 1, JSD_SDO_DATA_FLOAT,
          (void*)&drive_max_current)) {
    return 0;
  }
  MSG("EGD[%d] Drive Maximum Current is %f amps", slave_id, drive_max_current);

  if (config->egd.drive_max_current_limit == -FLT_MAX) {
    MSG("EGD[%d] Drive max current check overridden", slave_id);
  } else {
    if (config->egd.peak_current_limit > drive_max_current) {
      ERROR("EGD[%d] Peak Current (%f) cannot exceed Drive max current (%f)",
            slave_id, config->egd.peak_current_limit, drive_max_current);
      return 0;
    }
  }

  if (config->egd.continuous_current_limit > config->egd.peak_current_limit) {
    ERROR("EGD[%d] Continuous Current (%f) cannot exceed Peak Current(%f)",
          slave_id, config->egd.continuous_current_limit,
          config->egd.peak_current_limit);
    return 0;
  }

  return 1;
}

void jsd_egd_read_PDO_data(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);
  assert(sizeof(jsd_egd_txpdo_data_t) ==
         self->ecx_context.slavelist[slave_id].Ibytes);

  // memcpy the data from SOEM's IOmap to our egd txpdo state
  memcpy(&self->slave_states[slave_id].egd.txpdo,
         self->ecx_context.slavelist[slave_id].inputs,
         self->ecx_context.slavelist[slave_id].Ibytes);
}

void jsd_egd_write_PDO_data(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  if (self->slave_configs[slave_id].egd.drive_cmd_mode ==
      JSD_EGD_DRIVE_CMD_MODE_CS) {
    assert(sizeof(jsd_egd_rxpdo_data_cs_mode_t) ==
           self->ecx_context.slavelist[slave_id].Obytes);

    // memcpy the data from our egd rxpdo state to SOEM's IOmap
    memcpy(self->ecx_context.slavelist[slave_id].outputs,
           &self->slave_states[slave_id].egd.rxpdo_cs,
           self->ecx_context.slavelist[slave_id].Obytes);

  } else if (self->slave_configs[slave_id].egd.drive_cmd_mode ==
             JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    assert(sizeof(jsd_egd_rxpdo_data_profiled_mode_t) ==
           self->ecx_context.slavelist[slave_id].Obytes);

    // memcpy the data from our egd rxpdo state to SOEM's IOmap
    memcpy(self->ecx_context.slavelist[slave_id].outputs,
           &self->slave_states[slave_id].egd.rxpdo_prof,
           self->ecx_context.slavelist[slave_id].Obytes);

  } else {
    ERROR("bad drive command mode: %d",
          self->slave_configs[slave_id].egd.drive_cmd_mode);
  }
}

void jsd_egd_async_sdo_process(jsd_t* self, uint16_t slave_id) {
  jsd_slave_state_t* state = &self->slave_states[slave_id];

  while (!jsd_sdo_req_cirq_is_empty(&self->jsd_sdo_res_cirq[slave_id])) {
    jsd_sdo_req_t req = jsd_sdo_req_cirq_pop(&self->jsd_sdo_res_cirq[slave_id]);
    state->num_async_sdo_responses++;

    // 1) check if SDO operation was success
    // NOTE: Latch jsd fault type handled by EMCY message
    if (!req.success) {
      ERROR("Slave[%u] Failed last SDO operation on 0x%X:%u, wkc = %d",
            slave_id, req.sdo_index, req.sdo_subindex, req.wkc);
    }
    // NOTE: If there is a usecase for low-frequency SDO
    // Reads, EGD could parse SDO Read requests
    // and update public state data here
  }

  // 2) Check if any SDO operations are ongoing and update state
  state->egd.pub.async_sdo_in_prog =
      state->num_async_sdo_requests != state->num_async_sdo_responses;

  // 3) Check if async_sdo_in_prog has been held high for too long, indicating
  //    a lost SDO Request likely due to a queue overflow
  if (state->egd.last_async_sdo_in_prog == 0 &&
      state->egd.pub.async_sdo_in_prog == 1) {
    state->egd.async_sdo_in_prog_start_time = jsd_get_time_sec();
  }

  if (state->egd.pub.async_sdo_in_prog) {
    if (jsd_get_time_sec() - state->egd.async_sdo_in_prog_start_time >
        JSD_EGD_ASYNC_SDO_TIMEOUT_SEC) {
      if (state->egd.pub.fault_code != JSD_EGD_FAULT_SDO_ERROR) {
        // only print this one to not spew to stdout
        ERROR(
            "Async SDO has been in progress >%lf sec, likely SDO queue "
            "overflow. Latching fault.",
            JSD_EGD_ASYNC_SDO_TIMEOUT_SEC);
      }
      state->egd.pub.fault_code = JSD_EGD_FAULT_SDO_ERROR;
    }
  }

  state->egd.last_async_sdo_in_prog = state->egd.pub.async_sdo_in_prog;
}

void jsd_egd_update_state_from_PDO_data(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  jsd_egd_private_state_t* state = &self->slave_states[slave_id].egd;

  // drive position parameters
  state->pub.actual_position = state->txpdo.actual_position;
  state->pub.actual_velocity = state->txpdo.velocity_actual_value;
  state->pub.actual_current  = (double)state->txpdo.current_actual_value *
                              (double)state->motor_rated_current / 1e6;

  if (self->slave_configs[slave_id].egd.drive_cmd_mode ==
      JSD_EGD_DRIVE_CMD_MODE_CS) {
    state->pub.cmd_position = state->rxpdo_cs.target_position;
    state->pub.cmd_velocity = state->rxpdo_cs.target_velocity;
    state->pub.cmd_current  = (double)state->rxpdo_cs.target_torque *
                             (double)state->motor_rated_current / 1e6;

    state->pub.cmd_ff_position = state->rxpdo_cs.position_offset;
    state->pub.cmd_ff_velocity = state->rxpdo_cs.velocity_offset;
    state->pub.cmd_ff_current  = (double)state->rxpdo_cs.torque_offset *
                                (double)state->motor_rated_current / 1e6;
    state->pub.cmd_max_current = (double)(state->rxpdo_cs.max_current *
                                          (double)state->motor_rated_current) /
                                 1e6;

  } else if (self->slave_configs[slave_id].egd.drive_cmd_mode ==
             JSD_EGD_DRIVE_CMD_MODE_PROFILED) {
    state->pub.cmd_position = state->rxpdo_prof.target_position;
    state->pub.cmd_velocity = state->rxpdo_prof.target_velocity;
    state->pub.cmd_current  = (double)state->rxpdo_prof.target_torque *
                             (double)state->motor_rated_current / 1e6;

    state->pub.cmd_ff_position = 0;
    state->pub.cmd_ff_velocity = 0;
    state->pub.cmd_ff_current  = 0;

    state->pub.cmd_max_current = (double)(state->rxpdo_prof.max_current *
                                          (double)state->motor_rated_current) /
                                 1e6;

  } else {
    ERROR("bad drive command mode: %d",
          self->slave_configs[slave_id].egd.drive_cmd_mode);
  }

  // State Machine State with smart printing
  state->pub.actual_state_machine_state =
      state->txpdo.statusword & JSD_EGD_STATE_MACHINE_STATE_BITMASK;

  if (state->pub.actual_state_machine_state !=
      state->last_state_machine_state) {
    MSG("EGD[%d] actual State Machine State changed to %s (0x%x)", slave_id,
        jsd_egd_state_machine_state_to_string(
            state->pub.actual_state_machine_state),
        state->pub.actual_state_machine_state);
  }

  state->last_state_machine_state = state->pub.actual_state_machine_state;

  // Mode of Operation with smart printing
  state->pub.actual_mode_of_operation = state->txpdo.mode_of_operation_display;
  if (state->pub.actual_mode_of_operation !=
      state->last_actual_mode_of_operation) {
    MSG("EGD[%d] actual Mode of Operation changed to %s (0x%x)", slave_id,
        jsd_egd_mode_of_operation_to_string(
            state->pub.actual_mode_of_operation),
        state->pub.actual_mode_of_operation);
  }
  state->last_actual_mode_of_operation = state->pub.actual_mode_of_operation;

  state->pub.warning = state->txpdo.statusword >> 7 & 0x01;  // MAGIC TODO
  state->pub.target_reached =
      state->txpdo.statusword >> 10 & 0x01;  // MAGIC TODO

  // Status Register states
  state->servo_enabled =
      state->txpdo.status_register >> 4 & 0x01;  // MAGIC TODO
  state->fault_occured_when_enabled =
      state->txpdo.status_register >> 6 & 0x01;  // MAGIC TODO
  state->pub.sto_engaged =
      !(state->txpdo.status_register >> 14 & 0x01);  // TODO
  state->pub.motor_on =
      state->txpdo.status_register >> 22 & 0x01;  // MAGIC TODO
  state->pub.in_motion =
      state->txpdo.status_register >> 23 & 0x01;  // MAGIC TODO
  state->pub.hall_state =
      state->txpdo.status_register >> 24 & 0x07;  // TODO test me

  // STO status from status register with smart printing
  if (state->last_sto_engaged != state->pub.sto_engaged) {
    if (state->pub.sto_engaged) {
      ERROR("STO is engaged");
    } else {
      SUCCESS("STO is released");
    }
  }
  state->last_sto_engaged = state->pub.sto_engaged;

  // Digital Inputs
  state->interlock = state->txpdo.digital_inputs >> 3 & 0x01;  // MAGIC TODO
  int i;
  for (i = 0; i < JSD_EGD_NUM_DIGITAL_INPUTS; ++i) {
    state->pub.digital_inputs[i] =
        state->txpdo.digital_inputs >> (16 + i) & 0x01;
  }

  // bus voltage
  state->pub.bus_voltage =
      (double)state->txpdo.dc_link_circuit_voltage / 1000.0;

  // analog input voltage
  state->pub.analog_input_voltage = (double)state->txpdo.analog_input / 1000.0;

  // drive temp
  state->pub.drive_temperature = state->txpdo.drive_temperature_deg_c;
}
void jsd_egd_process_state_machine(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  ec_mbxbuft               MbxIn;
  ec_errort                error;
  jsd_egd_private_state_t* state = &self->slave_states[slave_id].egd;

  switch (state->pub.actual_state_machine_state) {
    case JSD_EGD_STATE_MACHINE_STATE_NOT_READY_TO_SWITCH_ON:
      // no-op
      break;
    case JSD_EGD_STATE_MACHINE_STATE_SWITCH_ON_DISABLED:
      set_controlword(self, slave_id,
                      JSD_EGD_STATE_MACHINE_CONTROLWORD_SHUTDOWN);
      break;
    case JSD_EGD_STATE_MACHINE_STATE_READY_TO_SWITCH_ON:
      set_controlword(self, slave_id,
                      JSD_EGD_STATE_MACHINE_CONTROLWORD_SWITCH_ON);
      break;
    case JSD_EGD_STATE_MACHINE_STATE_SWITCHED_ON:
      // STO drops us here
      // Handle reset
      if (state->new_reset) {
        set_controlword(self, slave_id,
                        JSD_EGD_STATE_MACHINE_CONTROLWORD_ENABLE_OPERATION);
      }
      break;
    case JSD_EGD_STATE_MACHINE_STATE_OPERATION_ENABLED:

      // Handle halt or fault
      if (state->new_halt_command ||
          state->pub.fault_code != JSD_EGD_FAULT_OKAY) {
        uint16_t cw = get_controlword(self, slave_id);
        cw &= ~(0x01 << 2);  // Quickstop
        set_controlword(self, slave_id, cw);

        state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_PROF_POS;
        set_mode_of_operation(self, slave_id,
                              state->requested_mode_of_operation);
        break;
      }

      set_controlword(self, slave_id,
                      JSD_EGD_STATE_MACHINE_CONTROLWORD_ENABLE_OPERATION);

      jsd_egd_process_mode_of_operation(self, slave_id);

      break;
    case JSD_EGD_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE:
      set_controlword(self, slave_id,
                      JSD_EGD_STATE_MACHINE_CONTROLWORD_ENABLE_OPERATION);
      break;
    case JSD_EGD_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE:
      set_controlword(self, slave_id,
                      JSD_EGD_STATE_MACHINE_CONTROLWORD_FAULT_RESET);
      break;
    case JSD_EGD_STATE_MACHINE_STATE_FAULT:
      state->new_reset = false;

      ecx_mbxreceive(&self->ecx_context, slave_id, (ec_mbxbuft*)&MbxIn, 0);
      if (ecx_iserror(&self->ecx_context)) {
        ecx_poperror(&self->ecx_context, &error);
        ERROR("EMCY: on slave id: %d, Description:  %s", error.Slave,
              jsd_egd_fault_code_to_string(
                  jsd_egd_get_fault_code_from_ec_error(error)));
        state->pub.fault_code = error.ErrorCode;
      }

      set_controlword(self, slave_id,
                      JSD_EGD_STATE_MACHINE_CONTROLWORD_FAULT_RESET);
      break;
    default:
      ERROR("EGD[%d] Unknown State Machine State: 0x%x", slave_id,
            state->pub.actual_state_machine_state);
  }

  if (state->new_reset) {
    state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_PROF_POS;
    set_mode_of_operation(self, slave_id, state->requested_mode_of_operation);
    state->new_halt_command = false;
    state->pub.fault_code   = JSD_EGD_FAULT_OKAY;
  }
  state->new_reset          = false;
  state->new_motion_command = false;
}

void jsd_egd_mode_of_op_handle_prof_pos(jsd_t* self, uint16_t slave_id) {
  jsd_egd_private_state_t* state  = &self->slave_states[slave_id].egd;
  jsd_egd_config_t*        config = &self->slave_configs[slave_id].egd;
  jsd_egd_motion_command_t cmd    = state->motion_command;

  switch (config->drive_cmd_mode) {
    case JSD_EGD_DRIVE_CMD_MODE_PROFILED: {
      state->rxpdo_prof.target_position  = cmd.prof_pos.target_position;
      state->rxpdo_prof.target_velocity  = 0;
      state->rxpdo_prof.target_torque    = 0;
      state->rxpdo_prof.profile_velocity = cmd.prof_pos.profile_velocity;
      state->rxpdo_prof.end_velocity     = cmd.prof_pos.end_velocity;
      state->rxpdo_prof.profile_accel    = cmd.prof_pos.profile_accel;
      state->rxpdo_prof.profile_decel    = cmd.prof_pos.profile_decel;

      if (state->new_motion_command) {
        // Bit 4: new setpoint
        state->rxpdo_prof.controlword |= (0x01 << 4);
      }

      // Bit 5: Change setpoint immediately
      state->rxpdo_prof.controlword |= (0x01 << 5);
      // Bit 6: Relative motion
      state->rxpdo_prof.controlword |= (cmd.prof_pos.relative << 6);
      break;
    }
    default:
      if (state->new_motion_command) {
        ERROR("Drive cmd mode: %d not suitable for profiled position command",
              config->drive_cmd_mode);
      }
      state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_DISABLED;
      break;
  }

  // This state is different from other modes of operations as there
  // is no true DISABLED mode
  set_mode_of_operation(self, slave_id, JSD_EGD_MODE_OF_OPERATION_PROF_POS);
}

void jsd_egd_mode_of_op_handle_prof_vel(jsd_t* self, uint16_t slave_id) {
  jsd_egd_private_state_t* state  = &self->slave_states[slave_id].egd;
  jsd_egd_config_t*        config = &self->slave_configs[slave_id].egd;
  jsd_egd_motion_command_t cmd    = state->motion_command;

  switch (config->drive_cmd_mode) {
    case JSD_EGD_DRIVE_CMD_MODE_PROFILED: {
      state->rxpdo_prof.target_position  = 0;
      state->rxpdo_prof.target_velocity  = cmd.prof_vel.target_velocity;
      state->rxpdo_prof.target_torque    = 0;
      state->rxpdo_prof.profile_velocity = 0;
      state->rxpdo_prof.end_velocity     = 0;
      state->rxpdo_prof.profile_accel    = cmd.prof_vel.profile_accel;
      state->rxpdo_prof.profile_decel    = cmd.prof_vel.profile_decel;

      set_mode_of_operation(self, slave_id, JSD_EGD_MODE_OF_OPERATION_PROF_VEL);

      break;
    }
    default:
      if (state->new_motion_command) {
        ERROR("Drive cmd mode: %d not suitable for profiled velocity command",
              config->drive_cmd_mode);
      }
      state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_DISABLED;
  }
}
void jsd_egd_mode_of_op_handle_prof_torque(jsd_t* self, uint16_t slave_id) {
  jsd_egd_private_state_t* state  = &self->slave_states[slave_id].egd;
  jsd_egd_config_t*        config = &self->slave_configs[slave_id].egd;
  jsd_egd_motion_command_t cmd    = state->motion_command;

  int16_t target_torque;

  switch (config->drive_cmd_mode) {
    case JSD_EGD_DRIVE_CMD_MODE_PROFILED: {
      target_torque =
          cmd.prof_torque.target_torque_amps * 1e6 / state->motor_rated_current;

      state->rxpdo_prof.target_position  = 0;
      state->rxpdo_prof.target_velocity  = 0;
      state->rxpdo_prof.target_torque    = target_torque;
      state->rxpdo_prof.profile_velocity = 0;
      state->rxpdo_prof.end_velocity     = 0;
      state->rxpdo_prof.profile_accel    = 0;
      state->rxpdo_prof.profile_decel    = 0;

      set_mode_of_operation(self, slave_id,
                            JSD_EGD_MODE_OF_OPERATION_PROF_TORQUE);

      break;
    }

    default:
      if (state->new_motion_command) {
        ERROR("Drive cmd mode: %d not suitable for profiled torque command",
              config->drive_cmd_mode);
      }
      state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_DISABLED;
  }
}

void jsd_egd_mode_of_op_handle_csp(jsd_t* self, uint16_t slave_id) {
  jsd_egd_private_state_t* state  = &self->slave_states[slave_id].egd;
  jsd_egd_config_t*        config = &self->slave_configs[slave_id].egd;
  jsd_egd_motion_command_t cmd    = state->motion_command;

  switch (config->drive_cmd_mode) {
    case JSD_EGD_DRIVE_CMD_MODE_CS: {
      state->rxpdo_cs.target_position = cmd.csp.target_position;
      state->rxpdo_cs.position_offset = cmd.csp.position_offset;
      state->rxpdo_cs.target_velocity = 0;
      state->rxpdo_cs.velocity_offset = cmd.csp.velocity_offset;
      state->rxpdo_cs.target_torque   = 0;
      state->rxpdo_cs.torque_offset =
          cmd.csp.torque_offset_amps * 1e6 / state->motor_rated_current;

      set_mode_of_operation(self, slave_id, JSD_EGD_MODE_OF_OPERATION_CSP);

      break;
    }
    default:
      if (state->new_motion_command) {
        ERROR("Drive cmd mode: %d not suitable for CSP command",
              config->drive_cmd_mode);
      }
      state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_DISABLED;
  }
}

void jsd_egd_mode_of_op_handle_csv(jsd_t* self, uint16_t slave_id) {
  jsd_egd_private_state_t* state  = &self->slave_states[slave_id].egd;
  jsd_egd_config_t*        config = &self->slave_configs[slave_id].egd;
  jsd_egd_motion_command_t cmd    = state->motion_command;

  switch (config->drive_cmd_mode) {
    case JSD_EGD_DRIVE_CMD_MODE_CS: {
      state->rxpdo_cs.target_position = 0;
      state->rxpdo_cs.position_offset = 0;
      state->rxpdo_cs.target_velocity = cmd.csv.target_velocity;
      state->rxpdo_cs.velocity_offset = cmd.csv.velocity_offset;
      state->rxpdo_cs.target_torque   = 0;
      state->rxpdo_cs.torque_offset =
          cmd.csv.torque_offset_amps * 1e6 / state->motor_rated_current;
      set_mode_of_operation(self, slave_id, JSD_EGD_MODE_OF_OPERATION_CSV);

      break;
    }
    default:
      if (state->new_motion_command) {
        ERROR("Drive cmd mode: %d not suitable for CSV command",
              config->drive_cmd_mode);
      }
      state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_DISABLED;
  }
}

void jsd_egd_mode_of_op_handle_cst(jsd_t* self, uint16_t slave_id) {
  jsd_egd_private_state_t* state  = &self->slave_states[slave_id].egd;
  jsd_egd_config_t*        config = &self->slave_configs[slave_id].egd;
  jsd_egd_motion_command_t cmd    = state->motion_command;

  switch (config->drive_cmd_mode) {
    case JSD_EGD_DRIVE_CMD_MODE_CS: {
      state->rxpdo_cs.target_position = 0;
      state->rxpdo_cs.position_offset = 0;
      state->rxpdo_cs.target_velocity = 0;
      state->rxpdo_cs.velocity_offset = 0;
      state->rxpdo_cs.target_torque =
          cmd.cst.target_torque_amps * 1e6 / state->motor_rated_current;
      state->rxpdo_cs.torque_offset =
          cmd.cst.torque_offset_amps * 1e6 / state->motor_rated_current;
      set_mode_of_operation(self, slave_id, JSD_EGD_MODE_OF_OPERATION_CST);

      break;
    }
    default:
      if (state->new_motion_command) {
        ERROR("Drive cmd mode: %d not suitable for CST command",
              config->drive_cmd_mode);
      }
      state->requested_mode_of_operation = JSD_EGD_MODE_OF_OPERATION_DISABLED;
  }
}

void jsd_egd_process_mode_of_operation(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EGD_PRODUCT_CODE);

  jsd_egd_private_state_t* state = &self->slave_states[slave_id].egd;

  // print out mode of operation change
  if (state->last_requested_mode_of_operation !=
      state->requested_mode_of_operation) {
    MSG("EGD[%d] Requested Mode of Operation changed to: %s", slave_id,
        jsd_egd_mode_of_operation_to_string(
            state->requested_mode_of_operation));
    if (!(state->requested_mode_of_operation ==
          JSD_EGD_MODE_OF_OPERATION_PROF_POS) &&
        state->pub.in_motion) {
      WARNING("EGD[%d] Drive is in motion, changing op mode is not advisable",
              slave_id);
    }
  }
  state->last_requested_mode_of_operation = state->requested_mode_of_operation;

  switch (state->requested_mode_of_operation) {
    case JSD_EGD_MODE_OF_OPERATION_DISABLED:
      break;

    case JSD_EGD_MODE_OF_OPERATION_PROF_POS:
      jsd_egd_mode_of_op_handle_prof_pos(self, slave_id);
      break;

    case JSD_EGD_MODE_OF_OPERATION_PROF_VEL:
      jsd_egd_mode_of_op_handle_prof_vel(self, slave_id);
      break;

    case JSD_EGD_MODE_OF_OPERATION_PROF_TORQUE:
      jsd_egd_mode_of_op_handle_prof_torque(self, slave_id);
      break;

    case JSD_EGD_MODE_OF_OPERATION_CSP:
      jsd_egd_mode_of_op_handle_csp(self, slave_id);
      break;

    case JSD_EGD_MODE_OF_OPERATION_CSV:
      jsd_egd_mode_of_op_handle_csv(self, slave_id);
      break;

    case JSD_EGD_MODE_OF_OPERATION_CST:
      jsd_egd_mode_of_op_handle_cst(self, slave_id);
      break;

    default:

      if (state->last_requested_mode_of_operation !=
          state->requested_mode_of_operation) {
        ERROR("Control mode: 0x%x (str: %s) not implemented",
              state->requested_mode_of_operation,
              jsd_egd_mode_of_operation_to_string(
                  state->requested_mode_of_operation));
      }
  }
}

jsd_egd_fault_code_t jsd_egd_get_fault_code_from_ec_error(ec_errort error) {
  // WARNING("ec_err_type: %d", error.Etype);

  if (error.Etype == EC_ERR_TYPE_SDO_ERROR) {
    return JSD_EGD_FAULT_SDO_ERROR;
  }

  switch (error.ErrorCode) {
    case 0x1000:
      return JSD_EGD_FAULT_RESERVED;
      break;
    case 0x2311:
      return JSD_EGD_FAULT_OVER_CURRENT;
      break;
    case 0x2340:
      return JSD_EGD_FAULT_SHORT_CIRCUIT;
      break;
    case 0x3120:
      return JSD_EGD_FAULT_UNDER_VOLTAGE;
      break;
    case 0x3130:
      return JSD_EGD_FAULT_LOSS_OF_PHASE;
      break;
    case 0x3310:
      return JSD_EGD_FAULT_OVER_VOLTAGE;
      break;
    case 0x4310:
      return JSD_EGD_FAULT_DRIVE_OVER_TEMP;
      break;
    case 0x5280:
      return JSD_EGD_FAULT_ECAM_DIFF;
      break;
    case 0x5281:
      return JSD_EGD_FAULT_TIMING_ERROR;
      break;
    case 0x5441:
      return JSD_EGD_FAULT_MOTOR_DISABLED_BY_SWITCH;
      break;

    case 0x5442:
      return JSD_EGD_FAULT_ABORT_MOTION;
      break;
    case 0x6180:
      return JSD_EGD_FAULT_CPU_STACK_OVERFLOW;
      break;
    case 0x6181:
      return JSD_EGD_FAULT_CPU_FATAL_EXCEPTION;
      break;
    case 0x6200:
      return JSD_EGD_FAULT_USER_PROG_ABORTED;
      break;
    case 0x6300:
      return JSD_EGD_FAULT_RPDO_MAP_ERROR;
      break;
    case 0x6320:
      return JSD_EGD_FAULT_INCONSISTENT_DATABASE;
      break;
    case 0x7121:
      return JSD_EGD_FAULT_MOTOR_STUCK;
      break;
    case 0x7300:
      return JSD_EGD_FAULT_FEEDBACK_ERROR;
      break;
    case 0x7306:
      return JSD_EGD_FAULT_COMMUTATION_FAILED;
      break;
    case 0x7380:
      return JSD_EGD_FAULT_FEEBACK_LOSS;
      break;
    case 0x7381:
      return JSD_EGD_FAULT_DIGITAL_HALL_BAD_CHANGE;
      break;
    case 0x7382:
      return JSD_EGD_FAULT_COMMUTATION_PROCESS_FAIL;
      break;

    case 0x8110:
      return JSD_EGD_FAULT_CAN_MSG_LOST;
      break;
    case 0x8130:
      return JSD_EGD_FAULT_HEARTBEAT_EVENT;
      break;
    case 0x8140:
      return JSD_EGD_FAULT_RECOVER_BUS_OFF;
      break;
    case 0x8200:
      return JSD_EGD_FAULT_NMT_PROTOCOL_ERROR;
      break;
    case 0x8210:
      return JSD_EGD_FAULT_ACCESS_UNCONFIGURED_RPDO;
      break;
    case 0x8311:
      return JSD_EGD_FAULT_PEAK_CURRENT_EXCEEDED;
      break;
    case 0x8380:
      return JSD_EGD_FAULT_FAILED_ELECTRICAL_ZERO;
      break;
    case 0x8381:
      return JSD_EGD_FAULT_CANNOT_TUNE;
      break;
    case 0x8480:
      return JSD_EGD_FAULT_SPEED_TRACKING_ERROR;
      break;
    case 0x8481:
      return JSD_EGD_FAULT_SPEED_LIMIT_EXCEEDED;
      break;

    case 0x8611:
      return JSD_EGD_FAULT_POSITION_TRACKING_ERROR;
      break;
    case 0x8680:
      return JSD_EGD_FAULT_POSITION_LIMIT_EXCEEDED;
      break;
    case 0xFF00:
      return JSD_EGD_FAULT_BAD_DATA_0XFF00;
      break;

    case 0xFF01:
      return JSD_EGD_FAULT_USER_PROG_EMIT;
      break;
    case 0xFF02:
      return JSD_EGD_FAULT_BAD_DATA_0XFF02;
      break;
    case 0xFF10:
      return JSD_EGD_FAULT_CANNOT_START_MOTOR;
      break;
    case 0xFF20:
      return JSD_EGD_FAULT_STO_ENGAGED;
      break;
    case 0xFF30:
      return JSD_EGD_FAULT_MODULO_OVERFLOW;
      break;
    case 0xFF34:
      return JSD_EGD_FAULT_NUMERIC_OVERFLOW;
      break;
    case 0xFF40:
      return JSD_EGD_FAULT_GANTRY_SLAVE_DISABLED;
      break;

    default:
      return JSD_EGD_FAULT_UNKNOWN;
      break;
  }
  return JSD_EGD_FAULT_UNKNOWN;
}
