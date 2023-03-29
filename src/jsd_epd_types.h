#ifndef JSD_EPD_TYPES_H
#define JSD_EPD_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"
#include "jsd/jsd_elmo_common_types.h"

#define JSD_EPD_PRODUCT_CODE (uint32_t)0x01100002

// TODO(dloret): Consider making the types related to DS-402 common between the
// EGD and EPD drivers:
// - JSD_*_NUM_DIGITAL_(OUTPUTS|INPUTS)
// - jsd_*_mode_of_operation_t

// NOTE: Used numbers specified in DS-402
#define JSD_EPD_NUM_DIGITAL_OUTPUTS 6
#define JSD_EPD_NUM_DIGITAL_INPUTS 16

// TODO(dloret): Double check whether RESET_DERATE is still necessary.
#define JSD_EPD_RESET_DERATE_SEC 1.0

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

/**
 * @brief Union of all motion commands
 */
typedef struct {
  union {
    jsd_elmo_motion_command_csp_t         csp;
    jsd_elmo_motion_command_csv_t         csv;
    jsd_elmo_motion_command_cst_t         cst;
    jsd_elmo_motion_command_prof_pos_t    prof_pos;
    jsd_elmo_motion_command_prof_vel_t    prof_vel;
    jsd_elmo_motion_command_prof_torque_t prof_torque;
  };
} jsd_epd_motion_command_t;

/**
 * @brief Elmo Platinum Drive JSD Configuration
 */
typedef struct {
  // TODO(dloret): Verify that selected integer is appropriate for the type
  // specified in command reference. Parameters set via SDO
  double
      max_motor_speed;  ///< Saturation value (cnts/s) of speed command to speed
                        ///< controller. Converted to 0x6080 using P_CA[18].
  uint8_t loop_period_ms;  ///< Expected loop period in ms for synchronous
                           ///< modes. 0x60C2.
  double torque_slope;  ///< A/s. Ignored by CS mode. Converted to 0x6087 using
                        ///< P_CL[1].

  // Parameters set via Two-Letter-Command (TLC)
  // IMPORTANT: Elmo alias objects are different between Gold and Platinum
  // lines.
  // TODO(dloret): Consider removing max_profile_accel and max_profile_decel
  // in both Gold and Platinum drivers. AC[1] and DC[1] are only relevant at
  // power-up when their values are assigned to objects 0x6083 and 0x6084,
  // respectively. So setting AC[1] and DC[1] at initialization of the driver
  // does not have an effect. Additionally, 0x6083 and 0x6084 are mapped to the
  // RxPDO.
  double max_profile_accel;         ///< P_AC[1]. Ignored by CS mode.
  double max_profile_decel;         ///< P_DC[1]. Ignored by CS mode.
  double velocity_tracking_error;   ///< P_ER[2] cnts/s
  double position_tracking_error;   ///< P_ER[3] cnts
  float  peak_current_limit;        ///< P_PL[1] A
  float  peak_current_time;         ///< P_PL[2] s
  float  continuous_current_limit;  ///< P_CL[1] A, continuous current limit
  float  motor_stuck_current_level_pct;   ///< P_CL[2] pct[0,100], < 2 disables
                                          ///< motor stuck protection.
  float  motor_stuck_velocity_threshold;  ///< P_CL[3] cnts/s
  float  motor_stuck_timeout;             ///< P_CL[4] ms
  double over_speed_threshold;  ///< P_HL[2] cnts/s, 0 disables over speed
                                ///< protection.
  double low_position_limit;    ///< P_LL[3] cnts, should be <= 0.
  double high_position_limit;  ///< P_HL[3] cnts, P_HL[3] = P_LL[3] = 0 disables
                               ///< out of position limits protection.
  int16_t brake_engage_msec;   ///< BP[1] ms, [0, 1000]
  int16_t brake_disengage_msec;  ///< BP[2] ms, [0, 1000]

  // Verification parameters
  uint32_t crc;  ///< CZ[1], changes whenever drive parameters change
  // TODO(dloret): Consider whether it is worth to have a parameter that
  // determines whether peak current limit is compared against the drive's
  // maximum current as in EGD. A potential option is to simply warn if PL[1] >
  // MC[1].
  int32_t smooth_factor;  ///< SF[1] ms, [0, 2048*P_TS*P_HS/1000]

  jsd_elmo_gain_scheduling_mode_t
      ctrl_gain_scheduling_mode;  ///< P_GS[2]. Set to -1 to use mode currently
                                  ///< set in the drive.
} jsd_epd_config_t;

/**
 * @brief Elmo Platinum Drive JSD State Data
 */
typedef struct {
  int32_t actual_position;  ///< Actual position, cnt
  int32_t actual_velocity;  ///< Actual velocity, cnt/s
  double  actual_current;   ///< Actual motor current, A

  int32_t cmd_position;     ///< Commanded position, cnt
  int32_t cmd_velocity;     ///< Commanded velocity, cnt/s
  double  cmd_current;      ///< Commanded motor current, A
  double  cmd_max_current;  ///< Limit on current command, A

  int32_t cmd_ff_position;  ///< Commanded feed-forward position, cnt
  int32_t cmd_ff_velocity;  ///< Commanded feed-forward velocity, cnt/s
  double  cmd_ff_current;   ///< Commanded feed-forward current, A

  uint32_t cmd_prof_velocity;      ///< Commanded profile velocity, cnt/s
  uint32_t cmd_prof_end_velocity;  ///< Commanded profile end velocity, cnt/s
  uint32_t cmd_prof_accel;         ///< Commanded profile acceleration, cnt/s/s
  uint32_t cmd_prof_decel;         ///< Commanded profile deceleration, cnt/s/s

  jsd_elmo_state_machine_state_t actual_state_machine_state;
  jsd_epd_mode_of_operation_t   actual_mode_of_operation;

  uint8_t sto_engaged;    ///< Safe Torque Off (EStop) status
  uint8_t hall_state;     ///< 3 Hall channels (ABC) in first 3 bits.
  uint8_t motor_on;       ///< Whether the motor is enabled (powered).
  uint8_t in_motion;      ///< Whether motor is in motion.
  uint8_t servo_enabled;  ///< Whether the servo is enabled (commands can be
                          ///< processed). For transition into Operation Enabled
                          ///< state, servo_enabled is 1 after time to disengage
                          ///< brake elapses. For a transition into any of the
                          ///< Power Disabled states, servo_enabled is 0 after
                          ///< time to engage brake elapses.
  uint8_t warning;        ///< From statusword, bit 7
  uint8_t target_reached;  ///< From statusword, bit 10, mode dependent.
  bool setpoint_ack_rise;  ///< True on rising edge of bit 12 of statusword. In
                           ///< Profiled Position mode, target_reached
                           ///< corresponds to the most recent command only
                           ///< after the first rising edge of the set-point
                           ///< acknowledge bit after sending such command.
  jsd_epd_fault_code_t fault_code;  ///< Fault based on Emergency error code.
  uint16_t emcy_error_code;  ///< Emergency error codes.

  double  bus_voltage;           ///< Bus voltage, V
  double  analog_input_voltage;  ///< Analog input 1, V
  uint16_t analog_input_adc;      ///< Analog input 2, raw

  uint8_t digital_inputs[JSD_EPD_NUM_DIGITAL_INPUTS];
  uint8_t digital_output_cmd[JSD_EPD_NUM_DIGITAL_OUTPUTS];
  float   drive_temperature;  ///< deg C
} jsd_epd_state_t;

/**
 * @brief TxPDO struct used to read device data in SOEM IOmap
 *
 * Note: struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  int32_t actual_position;            ///< 0x6064
  int32_t velocity_actual_value;      ///< 0x6069
  int16_t current_actual_value;       ///< 0x6078
  int8_t  mode_of_operation_display;  ///< 0x6061
  uint32_t dc_link_circuit_voltage;  ///< 0x6079
  float    drive_temperature_deg_c;  ///< 0x3610
  uint32_t digital_inputs;           ///< 0x60FD
  int16_t  analog_input_1;           ///< 0x2205:01
  int16_t  analog_input_2;           ///< 0x2205:02
  uint32_t status_register_1;        ///< 0x3607:01
  uint32_t status_register_2;        ///< 0x3607:02
  uint16_t statusword;               ///< 0x6041
} jsd_epd_txpdo_data_t;

/**
 * @brief RxPDO struct used to set device command data in SOEM IOmap
 *
 * Note: struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  int32_t  target_position;    ///< 0x607A
  int32_t  target_velocity;    ///< 0x60FF
  int16_t  target_torque;      ///< 0x6071
  int32_t  position_offset;    ///< 0x60B0
  int32_t  velocity_offset;    ///< 0x60B1
  int16_t  torque_offset;      ///< 0x60B2
  int8_t   mode_of_operation;  ///< 0x6060
  uint16_t max_current;        ///< 0x6073
  // TODO(dloret): max_current
  // - Might need to remove since it does not have an effect on PL.
  // - EGD's Profile rxPDO maps max_current to 0x6072 (Max Torque) instead of
  //   0x6073 (Max Current). But they should be equal in this case.
  uint32_t digital_outputs;    ///< 0x60FE
  uint16_t controlword;  ///< 0x6040. NOTE(dloret): bit arrangement is different
                         ///< from Gold line.
  uint32_t profile_velocity;  ///< 0x6081
  uint32_t end_velocity;      ///< 0x6082
  uint32_t profile_accel;     ///< 0x6083
  uint32_t profile_decel;     ///< 0x6084
  int16_t  gain_scheduling_index;  ///< 0x36E0
} jsd_epd_rxpdo_data_t;

/**
 * @brief Elmo Platinum Drive JSD private state data
 *
 * The fields included here are for JSD's internal use and may not be helpful
 * for client applications.
 */
typedef struct {
  jsd_epd_state_t pub;  ///< Public state used by client applications.

  // SOEM PDO data
  jsd_epd_txpdo_data_t txpdo;  ///< Raw TxPDO data
  jsd_epd_rxpdo_data_t rxpdo;  ///< Raw RxPDO data

  uint32_t motor_rated_current;  ///< Set the same as continuous current limit
                                 ///< (CL[1]), mA

  // User input
  bool                        new_reset;
  bool                        new_halt_command;
  bool                        new_motion_command;
  jsd_epd_motion_command_t    motion_command;  ///< Last command from user
  jsd_epd_mode_of_operation_t requested_mode_of_operation;
  jsd_epd_mode_of_operation_t last_requested_mode_of_operation;
  double                      last_reset_time;

  uint8_t interlock;  ///< 1 when one or both of STO inputs are disabled.
  uint8_t fault_occured_when_enabled;  ///< From Status Register

  // State tracking
  jsd_elmo_state_machine_state_t last_state_machine_state;
  // TODO(dloret): Figure out debugging messages related to state changes
  // without affecting real-time guarantees.

  double
      fault_real_time;  /// Timestamp of the last transition into fault of the
                        /// drive's state machine. This is system time and is
                        /// needed to compare against the timestamp of the EMCY
                        /// error code because SOEM uses system time for it.
  double fault_mono_time;  /// Timestamp from monotonic clock of the last
                           /// transition into fault. This is used to measure a
                           /// timeout to receive the EMCY error code.

  uint8_t setpoint_ack;  ///< Setpoint ackowledge (Profiled Position mode),
                         ///< statustword, bit 12
  uint8_t last_setpoint_ack;
  bool prof_pos_waiting_setpoint_ack;  ///< When in Profiled Position mode, it
                                       ///< indicates whether the driver is
                                       ///< waiting for the drive to acknowledge
                                       ///< reception of a new profiled position
                                       ///< set-point so that the driver can
                                       ///< turn off the new set-point bit in
                                       ///< the controlword.
} jsd_epd_private_state_t;

#ifdef __cplusplus
}
#endif

#endif