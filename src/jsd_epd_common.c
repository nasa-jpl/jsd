#include "jsd/jsd_epd_common.h"

#include <stdlib.h>
#include <string.h>

#include "jsd/jsd_print.h"
#include "jsd/jsd_sdo.h"

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

uint16_t jsd_epd_lc_to_do_impl(char letter_command[2]) {
  jsd_epd_lc_pair_t  key        = {.lc_chars = letter_command};
  jsd_epd_lc_pair_t* found_pair = bsearch(
      &key, jsd_epd_lc_lookup_table,
      sizeof(jsd_epd_lc_lookup_table) / sizeof(jsd_epd_lc_lookup_table[0]),
      sizeof(jsd_epd_lc_lookup_table[0]), jsd_epd_compare_lc_keys);
  return found_pair ? found_pair->do_index : 0x0000;
}

void jsd_epd_async_sdo_set_drive_position_impl(jsd_t* self, uint16_t slave_id,
                                               double   position,
                                               uint16_t app_id) {
  jsd_sdo_set_param_async(self, slave_id, jsd_epd_lc_to_do_impl("PX"), 1,
                          JSD_SDO_DATA_DOUBLE, &position, app_id);
}

void jsd_epd_async_sdo_set_unit_mode_impl(jsd_t* self, uint16_t slave_id,
                                          int16_t mode, uint16_t app_id) {
  jsd_sdo_set_param_async(self, slave_id, jsd_epd_lc_to_do_impl("UM"), 1,
                          JSD_SDO_DATA_I16, &mode, app_id);
}

void jsd_epd_async_sdo_set_ctrl_gain_scheduling_mode_impl(
    jsd_t* self, uint16_t slave_id, jsd_elmo_gain_scheduling_mode_t mode,
    uint16_t app_id) {
  if (mode < 0 || mode > 68) {
    ERROR("Gain scheduling mode %d for the controller is not valid.", mode);
    return;
  }
  int64_t mode_i64 = mode;
  jsd_sdo_set_param_async(self, slave_id, jsd_epd_lc_to_do_impl("GS"), 2,
                          JSD_SDO_DATA_I64, &mode_i64, app_id);
}

const char* jsd_epd_fault_code_to_string_impl(jsd_epd_fault_code_t fault_code) {
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

bool jsd_epd_product_code_is_compatible_impl(uint32_t product_code) {
  return (product_code == JSD_EPD_PRODUCT_CODE_STD_FW) ||
         (product_code == JSD_EPD_PRODUCT_CODE_SAFETY_FW);
}
