#ifndef JSD_EPD_COMMON_TYPES_H
#define JSD_EPD_COMMON_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

// Currently JSD does not support the Functional Safety capability offered by
// some Platinum drive models. Such drives can still be run with this driver,
// but they will behave as the standard non-Safety drives.
// Product code of Platinum drives that use the standard non-Safety firmware
#define JSD_EPD_PRODUCT_CODE_STD_FW (uint32_t)0x00100002
// Product code of Platinum drives that use the Safety firmware (FSOE, Safe I/O)
#define JSD_EPD_PRODUCT_CODE_SAFETY_FW (uint32_t)0x01100002

// TODO(dloret): Double check whether RESET_DERATE is still necessary.
#define JSD_EPD_RESET_DERATE_SEC 1.0

#define JSD_EPD_MAX_BYTES_PDO_CHANNEL (128)

/**
 * @brief Elmo Platinum Drive Mode of Operation
 */
typedef enum {
  JSD_EPD_MODE_OF_OPERATION_DISABLED =
      0,  // TODO(dloret): might want to remove this if we no longer need a
          // drive command mode.
  JSD_EPD_MODE_OF_OPERATION_PROF_POS    = 1,
  JSD_EPD_MODE_OF_OPERATION_PROF_VEL    = 3,
  JSD_EPD_MODE_OF_OPERATION_PROF_TORQUE = 4,
  JSD_EPD_MODE_OF_OPERATION_CSP         = 8,
  JSD_EPD_MODE_OF_OPERATION_CSV         = 9,
  JSD_EPD_MODE_OF_OPERATION_CST         = 10
} jsd_epd_mode_of_operation_t;

/**
 * @brief Elmo Platinum Drive Fault Code
 *
 * See Platinum Administrator Guide, chapter 7
 */
typedef enum {
  // Make integers different from jsd_egd_fault_code_t so that fault codes are
  // easier to process in Fastcat
  JSD_EPD_FAULT_OKAY = 100,
  JSD_EPD_FAULT_SHORT_PROTECTION,
  JSD_EPD_FAULT_UNDER_VOLTAGE,
  JSD_EPD_FAULT_LOSS_OF_PHASE,
  JSD_EPD_FAULT_OVER_VOLTAGE,
  JSD_EPD_FAULT_MOTOR_OVER_TEMPERATURE,
  JSD_EPD_FAULT_DRIVE_OVER_TEMPERATURE,
  JSD_EPD_FAULT_GANTRY_YAW_ERROR_LIMIT_EXCEEDED,
  JSD_EPD_FAULT_EXTERNAL_INHIBIT_TRIGGERED,
  JSD_EPD_FAULT_ADDITIONAL_ABORT_ACTIVE,
  JSD_EPD_FAULT_VECTOR_ABORT,
  JSD_EPD_FAULT_RPDO_FAILED,
  JSD_EPD_FAULT_MOTOR_STUCK,
  JSD_EPD_FAULT_FEEDBACK_ERROR,
  JSD_EPD_FAULT_HALL_MAIN_FEEDBACK_MISMATCH,
  JSD_EPD_FAULT_HALL_BAD_CHANGE,
  JSD_EPD_FAULT_COMMUTATION_PROCESS_FAIL,
  JSD_EPD_FAULT_CAN_MESSAGE_LOST,
  JSD_EPD_FAULT_SYNC_OR_FRAME_LOSS,
  JSD_EPD_FAULT_RECOVERED_FROM_BUS_OFF,
  JSD_EPD_FAULT_ACCESS_NON_CONFIGURED_RPDO,
  JSD_EPD_FAULT_INCORRECT_RPDO_LENGTH,
  JSD_EPD_FAULT_PEAK_CURRENT_EXCEEDED,
  JSD_EPD_FAULT_SPEED_TRACKING_ERROR,
  JSD_EPD_FAULT_SPEED_LIMIT_EXCEEDED,
  JSD_EPD_FAULT_POSITION_TRACKING_ERROR,
  JSD_EPD_FAULT_POSITION_LIMIT_EXCEEDED,
  JSD_EPD_FAULT_CAN_INTERPOLATED_MODE_EMERGENCY,
  JSD_EPD_FAULT_CANNOT_START_MOTOR,
  JSD_EPD_FAULT_STO_ENGAGED,
  JSD_EPD_FAULT_MOTOR_DISABLE_COMMAND,
  JSD_EPD_FAULT_KINEMATICS_ERROR,
  JSD_EPD_FAULT_GANTRY_MASTER_ERROR,
  JSD_EPD_FAULT_GANTRY_SLAVE_DISABLED,
  JSD_EPD_FAULT_GANTRY_ATTACHED_SLAVE_FAULT,
  JSD_EPD_FAULT_UNKNOWN
} jsd_epd_fault_code_t;

#ifdef __cplusplus
}
#endif

#endif