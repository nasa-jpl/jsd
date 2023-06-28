#ifndef JSD_ELMO_COMMON_TYPES_H
#define JSD_ELMO_COMMON_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

/**
 * @brief Elmo drive State Machine's State
 */
typedef enum {
  JSD_ELMO_STATE_MACHINE_STATE_NOT_READY_TO_SWITCH_ON = 0X00,
  JSD_ELMO_STATE_MACHINE_STATE_SWITCH_ON_DISABLED     = 0X40,
  JSD_ELMO_STATE_MACHINE_STATE_READY_TO_SWITCH_ON     = 0x21,
  JSD_ELMO_STATE_MACHINE_STATE_SWITCHED_ON            = 0x23,
  JSD_ELMO_STATE_MACHINE_STATE_OPERATION_ENABLED      = 0x27,
  JSD_ELMO_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE      = 0x07,
  JSD_ELMO_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE  = 0x0F,
  JSD_ELMO_STATE_MACHINE_STATE_FAULT                  = 0x08
} jsd_elmo_state_machine_state_t;

/**
 * @brief Elmo drive's gain scheduling mode for the controller and filters.
 *
 * See Command Reference, command GS/P_GS[2, 16, 17, 18].
 *
 */
typedef enum {
  JSD_ELMO_GAIN_SCHEDULING_MODE_PRELOADED =
      -1,  ///< Scheduling mode currently set in the drive
  JSD_ELMO_GAIN_SCHEDULING_MODE_DISABLED = 0,   ///< No gain scheduling
  JSD_ELMO_GAIN_SCHEDULING_MODE_SPEED    = 64,  ///< Scheduling by speed
  JSD_ELMO_GAIN_SCHEDULING_MODE_POSITION = 65,  ///< Scheduling by position
  JSD_ELMO_GAIN_SCHEDULING_MODE_SETTLING = 66,  ///< Scheduling by Best Settling
  JSD_ELMO_GAIN_SCHEDULING_MODE_MANUAL_LOW =
      67,  ///< Manual selection via lower byte of 0x36E0 object
  JSD_ELMO_GAIN_SCHEDULING_MODE_MANUAL_HIGH =
      68,  ///< Manual selection via upper byte of 0x36E0 object
} jsd_elmo_gain_scheduling_mode_t;

/**
 * @brief Elmo drive's motion command for Cyclic Synchronous Position mode of
 * operation
 */
typedef struct {
  int32_t target_position;     ///< 0x607A
  int32_t position_offset;     ///< 0x60B0
  int32_t velocity_offset;     ///< 0x60B1
  double  torque_offset_amps;  ///< Converted to 0x60B2 using CL[1]
} jsd_elmo_motion_command_csp_t;

/**
 * @brief Elmo drive's motion command for Cyclic Synchronous Velocity mode of
 * operation
 */
typedef struct {
  int32_t target_velocity;     ///< 0x60FF
  int32_t velocity_offset;     ///< 0x60B1
  double  torque_offset_amps;  ///< Converted to 0x60B2 using CL[1]
} jsd_elmo_motion_command_csv_t;

/**
 * @brief Elmo drive's motion command for Cyclic Synchronous Torque mode of
 * operation
 */
typedef struct {
  double target_torque_amps;  ///< Converted to 0x6071 using CL[1]
  double torque_offset_amps;  ///< Converted to 0x60B2 using CL[1]
} jsd_elmo_motion_command_cst_t;

/**
 * @brief Elmo drive's motion command for Profiled Position mode of operation
 */
typedef struct {
  int32_t  target_position;   ///< 0x6074
  uint32_t profile_velocity;  ///< 0x6081
  uint32_t end_velocity;      ///< 0x6082
  uint32_t profile_accel;     ///< 0x6083
  uint32_t profile_decel;     ///< 0x6084
  uint8_t
      relative;  ///< target_position is relative to actuator's actual position.
} jsd_elmo_motion_command_prof_pos_t;

/**
 * @brief Elmo drive's motion command for Profiled Velocity mode of operation
 */
typedef struct {
  int32_t  target_velocity;  ///< 0x60FF
  uint32_t profile_accel;    ///< 0x6083
  uint32_t profile_decel;    ///< 0x6084
} jsd_elmo_motion_command_prof_vel_t;

/**
 * @brief Elmo drive's motion command for Profile Torque mode of operation
 */
typedef struct {
  double target_torque_amps;  ///< Converted to 0x6071 using CL[1]
} jsd_elmo_motion_command_prof_torque_t;

#ifdef __cplusplus
}
#endif

#endif