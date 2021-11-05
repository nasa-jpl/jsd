#ifndef JSD_EGD_TYPES_H
#define JSD_EGD_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EGD_PRODUCT_CODE (uint32_t)0x00030924

#define JSD_EGD_NUM_DIGITAL_OUTPUTS 6
#define JSD_EGD_NUM_DIGITAL_INPUTS 6

#define JSD_EGD_ASYNC_SDO_TIMEOUT_SEC 5.0

#define JSD_EGD_RESET_DERATE_SEC 1.0

/**
 * @brief Elmo Gold Drive Command Mode
 *
 * This configuration parameter controls the command-set available to the
 * device.
 *
 * if JSD_EGD_DRIVE_CMD_MODE_CD is set, the drive will ONLY accept
 * [CSP, CSV, CST, and digital_output commands].
 *
 * If JSD_EGD_DRIVE_CMD_MODE_PROFILED is set, the drive will ONLY accept
 * [prof_pos, prof_vel, and prof_torque] commands.
 *
 * Reset, Halt, and disable commands are always accepted by the drive. State
 * output
 * is NOT modified by the Command Mode.
 *
 * Note: This is only required due to a PDO mapping limitation of 32 Bytes and
 * SOEM's
 * SDO is not a suitable for use in real-time control loop, without a state-ful
 * process loop.
 */
typedef enum {
  JSD_EGD_DRIVE_CMD_MODE_PROFILED,  // uses onboard profiler
  JSD_EGD_DRIVE_CMD_MODE_CS,        // uses external profiler
} jsd_egd_drive_cmd_mode_t;

/**
 * @brief Elmo Gold Drive Mode of Operation
 *
 * Excluding modes of operation available by Elmo but not used by JSD.
 * See MAN-G-DS402 Section 8 (Object 0x6060)
 */
typedef enum {
  JSD_EGD_MODE_OF_OPERATION_DISABLED    = 0,   ///< disabled
  JSD_EGD_MODE_OF_OPERATION_PROF_POS    = 1,   ///< profiled position
  JSD_EGD_MODE_OF_OPERATION_PROF_VEL    = 3,   ///< profiled velocity
  JSD_EGD_MODE_OF_OPERATION_PROF_TORQUE = 4,   ///< profiled torque
  JSD_EGD_MODE_OF_OPERATION_CSP         = 8,   ///< cyclic syncronous position
  JSD_EGD_MODE_OF_OPERATION_CSV         = 9,   ///< cyclic syncronous velocity
  JSD_EGD_MODE_OF_OPERATION_CST         = 10,  ///< cyclic syncronous torque
} jsd_egd_mode_of_operation_t;

/**
 * @brief Elmo Gold Drive Statemachine State
 *
 * Determined by Bits 0-3, 5 and 6 from the statusword.
 * See MAN-G-DS402 Section 6.3
 */
typedef enum {
  JSD_EGD_STATE_MACHINE_STATE_NOT_READY_TO_SWITCH_ON = 0x00,
  JSD_EGD_STATE_MACHINE_STATE_SWITCH_ON_DISABLED     = 0x40,
  JSD_EGD_STATE_MACHINE_STATE_READY_TO_SWITCH_ON     = 0x21,
  JSD_EGD_STATE_MACHINE_STATE_SWITCHED_ON            = 0x23,
  JSD_EGD_STATE_MACHINE_STATE_OPERATION_ENABLED      = 0x27,
  JSD_EGD_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE      = 0x07,
  JSD_EGD_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE  = 0x0F,
  JSD_EGD_STATE_MACHINE_STATE_FAULT                  = 0x08,
} jsd_egd_state_machine_state_t;

/**
 * @brief Elmo Gold Drive Fault Code
 *
 * See MAN-G-CR Section MF for fault code table
 */
typedef enum {
  JSD_EGD_FAULT_OKAY = 0,
  JSD_EGD_FAULT_RESERVED,
  JSD_EGD_FAULT_OVER_CURRENT,
  JSD_EGD_FAULT_SHORT_CIRCUIT,
  JSD_EGD_FAULT_UNDER_VOLTAGE,
  JSD_EGD_FAULT_LOSS_OF_PHASE,
  JSD_EGD_FAULT_OVER_VOLTAGE,
  JSD_EGD_FAULT_DRIVE_OVER_TEMP,
  JSD_EGD_FAULT_ECAM_DIFF,
  JSD_EGD_FAULT_TIMING_ERROR,
  JSD_EGD_FAULT_MOTOR_DISABLED_BY_SWITCH,

  JSD_EGD_FAULT_ABORT_MOTION,
  JSD_EGD_FAULT_CPU_STACK_OVERFLOW,
  JSD_EGD_FAULT_CPU_FATAL_EXCEPTION,
  JSD_EGD_FAULT_USER_PROG_ABORTED,
  JSD_EGD_FAULT_RPDO_MAP_ERROR,
  JSD_EGD_FAULT_INCONSISTENT_DATABASE,
  JSD_EGD_FAULT_MOTOR_STUCK,
  JSD_EGD_FAULT_FEEDBACK_ERROR,
  JSD_EGD_FAULT_COMMUTATION_FAILED,
  JSD_EGD_FAULT_FEEBACK_LOSS,
  JSD_EGD_FAULT_DIGITAL_HALL_BAD_CHANGE,
  JSD_EGD_FAULT_COMMUTATION_PROCESS_FAIL,

  JSD_EGD_FAULT_CAN_MSG_LOST,
  JSD_EGD_FAULT_HEARTBEAT_EVENT,
  JSD_EGD_FAULT_RECOVER_BUS_OFF,
  JSD_EGD_FAULT_NMT_PROTOCOL_ERROR,
  JSD_EGD_FAULT_ACCESS_UNCONFIGURED_RPDO,
  JSD_EGD_FAULT_PEAK_CURRENT_EXCEEDED,
  JSD_EGD_FAULT_FAILED_ELECTRICAL_ZERO,
  JSD_EGD_FAULT_CANNOT_TUNE,
  JSD_EGD_FAULT_SPEED_TRACKING_ERROR,
  JSD_EGD_FAULT_SPEED_LIMIT_EXCEEDED,

  JSD_EGD_FAULT_POSITION_TRACKING_ERROR,
  JSD_EGD_FAULT_POSITION_LIMIT_EXCEEDED,
  JSD_EGD_FAULT_BAD_DATA_0XFF00,

  JSD_EGD_FAULT_USER_PROG_EMIT,
  JSD_EGD_FAULT_BAD_DATA_0XFF02,
  JSD_EGD_FAULT_CANNOT_START_MOTOR,
  JSD_EGD_FAULT_STO_ENGAGED,
  JSD_EGD_FAULT_MODULO_OVERFLOW,
  JSD_EGD_FAULT_NUMERIC_OVERFLOW,
  JSD_EGD_FAULT_GANTRY_SLAVE_DISABLED,

  JSD_EGD_FAULT_SDO_ERROR,

  JSD_EGD_FAULT_UNKNOWN,

} jsd_egd_fault_code_t;

/**
 * @brief Elmo Gold Drive Motion Command for Profiled Position Mode of
 * Operation
 *
 * See MAN-G-DS402 Section 12.1
 */
typedef struct {
  int32_t  target_position;   ///< 0x6074
  uint32_t profile_velocity;  ///< 0x6081
  uint32_t end_velocity;      ///< 0x6082
  uint32_t profile_accel;     ///< 0x6083
  uint32_t profile_decel;     ///< 0x6084
  uint8_t  relative;          ///< target_position is relative to act pos
} jsd_egd_motion_command_prof_pos_t;

/**
 * @brief Elmo Gold Drive Motion Command for Profiled Velocity Mode of
 * Operation
 *
 * See MAN-G-DS402 Section 17
 */
typedef struct {
  int32_t  target_velocity;  ///< 0x60FF
  uint32_t profile_accel;    ///< 0x6083
  uint32_t profile_decel;    ///< 0x6084
} jsd_egd_motion_command_prof_vel_t;

/**
 * @brief Elmo Gold Drive Motion Command for Profiled Torque Mode of
 * Operation
 *
 * See MAN-G-DS402 Section 18
 */
typedef struct {
  double target_torque_amps;  ///< converted to 0x6071 using CL[1]
} jsd_egd_motion_command_prof_torque_t;

/**
 * @brief Elmo Gold Drive Motion Command for Cyclic Synchronous Position Mode of
 * Operation
 *
 * See MAN-G-DS402 Section 14
 */
typedef struct {
  int32_t target_position;     ///< 0x6074
  int32_t position_offset;     ///< 0x60B0
  int32_t velocity_offset;     ///< 0x60B1
  double  torque_offset_amps;  ///< converted to 0x60B2 using CL[1]
} jsd_egd_motion_command_csp_t;

/**
 * @brief Elmo Gold Drive Motion Command for Cyclic Synchronous Velocity Mode of
 * Operation
 *
 * See MAN-G-DS402 Section 15
 */
typedef struct {
  int32_t target_velocity;     ///< 0x60FF
  int32_t velocity_offset;     ///< 0x60B1
  double  torque_offset_amps;  ///< converted to 0x60B2 using CL[1]
} jsd_egd_motion_command_csv_t;

/**
 * @brief Elmo Gold Drive Motion Command for Cyclic Synchronous Torque Mode of
 * Operation
 *
 * See MAN-G-DS402 Section 16
 */
typedef struct {
  double target_torque_amps;  ///< converted to 0x6071 using CL[1]
  double torque_offset_amps;  ///< converted to 0x60B2 using CL[1]
} jsd_egd_motion_command_cst_t;

/**
 * @brief Union of all motion commands
 */
typedef struct {
  union {
    jsd_egd_motion_command_prof_pos_t    prof_pos;
    jsd_egd_motion_command_prof_vel_t    prof_vel;
    jsd_egd_motion_command_prof_torque_t prof_torque;
    jsd_egd_motion_command_csp_t         csp;
    jsd_egd_motion_command_csv_t         csv;
    jsd_egd_motion_command_cst_t         cst;
  };
} jsd_egd_motion_command_t;

/**
 * @brief Elmo Gold Drive JSD Configuration
 */
typedef struct {
  jsd_egd_drive_cmd_mode_t
      drive_cmd_mode;  ///< controls which egd commands are supported

  // Parameters set via SDO
  double  max_motor_speed;  ///< max speed in cnts/sec, negative disables
  int8_t  loop_period_ms;   ///< expected loop period in ms for synch. modes
  double  torque_slope;  ///< amps/sec, converted to 0x6087 using CL[1]. Ignored
                         /// by CS mode

  // Parameters set via Two-Letter-Command (TLC)
  uint32_t max_profile_accel;         ///< AC[1] max accel, Ignored by CS mode
  uint32_t max_profile_decel;         ///< DC[1] max decel, Ignored by CS mode
  int32_t  velocity_tracking_error;   ///< ER[2] cnts/sec
  int32_t  position_tracking_error;   ///< ER[3] cnts
  float    peak_current_limit;        ///< PL[1] Amps
  float    peak_current_time;         ///< PL[2] sec
  float    continuous_current_limit;  ///< CL[1] Amps, ref current value
  float    motor_stuck_current_level_pct;  ///< CL[2] pct[0,100], < 2 disables
                                           /// motor stuck feature
  float   motor_stuck_velocity_threshold;  ///< CL[3] cnts/sec
  float   motor_stuck_timeout;             ///< CL[4] msec
  int32_t over_speed_threshold;  ///< HL[2] cnts, 0 disables over speed feature
  int32_t low_position_limit;    ///< LL[3] cnts, user input should be <= 0
  int32_t high_position_limit;   ///< HL[3] cnts, HL[3] = LL[3] = 0 disables pos
                                 /// limits
  int32_t brake_engage_msec;     ///< BP[1] msec, [0,1000]
  int32_t brake_disengage_msec;  ///< BP[2] msec, [0,1000]

  // Verification parameters
  int32_t crc;  ///< OV[52] updated whenever drive parameters change
  float drive_max_current_limit;  ///< MC, drive max current per product config.

  int32_t smooth_factor;  ///< SF[1] msec, [0, 63] 0 is default

} jsd_egd_config_t;

/**
 * @brief Elmo Gold Drive JSD State Data
 */
typedef struct {
  int32_t actual_position;  ///< actual position, counts
  int32_t actual_velocity;  ///< actual velocity, counts/sec
  double  actual_current;   ///< actual motor current converted to amps

  int32_t cmd_position;     ///< cmd position, counts
  int32_t cmd_velocity;     ///< cmd velocity, counts/sec
  double  cmd_current;      ///< cmd motor current converted to amps
  double  cmd_max_current;  ///< commanded torque to ELMO

  int32_t cmd_ff_position;  ///< cmd feed-forward position, counts
  int32_t cmd_ff_velocity;  ///< cmd feed-forward velocity, counts/sec
  double  cmd_ff_current;   ///< cmd feed-forwared current converted to amps

  jsd_egd_state_machine_state_t actual_state_machine_state;
  jsd_egd_mode_of_operation_t   actual_mode_of_operation;
  bool                          async_sdo_in_prog;

  uint8_t sto_engaged;     ///< Safe Torque Off (Estop) status
  uint8_t hall_state;      ///< 3 Hall channels (ABC) in first 3 bits TODO test
  uint8_t in_motion;       ///< if motor is in motion
  uint8_t warning;         ///< from statusword (SW), bit 7
  uint8_t target_reached;  ///< from SW, bit 10 mode dependent
  uint8_t motor_on;        ///< from SW, indicates brake and drive status
  jsd_egd_fault_code_t fault_code;  ///< from EMCY, != 0 indicates fault

  double   bus_voltage;           ///< bus voltage in volts
  double   analog_input_voltage;  ///< Analog 1 input in volts
  uint8_t  digital_inputs[JSD_EGD_NUM_DIGITAL_INPUTS];
  uint8_t  digital_output_cmd[JSD_EGD_NUM_DIGITAL_OUTPUTS];
  uint32_t drive_temperature;  ///< deg C

} jsd_egd_state_t;

/**
 * @brief TxPDO struct used to read device data in SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 * See G-ETHERCATAM Section 6.1
 */
typedef struct __attribute__((__packed__)) {
  int32_t  actual_position;            ///< 0x6064
  uint32_t digital_inputs;             ///< 0x60FD
  uint16_t statusword;                 ///< 0x6041
  int16_t  analog_input;               ///< 0x2205:01
  int32_t  velocity_actual_value;      ///< 0x6069
  int16_t  current_actual_value;       ///< 0x6078
  uint32_t status_register;            ///< 0x1002
  int8_t   mode_of_operation_display;  ///< 0x6061
  uint32_t dc_link_circuit_voltage;    ///< 0x6079
  uint32_t drive_temperature_deg_c;    ///< 0x2203 only works on newer firmware
} jsd_egd_txpdo_data_t;

/**
 * @brief RxPDO struct used to set device command data in SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 * See G-ETHERCATAM Section 6.1
 */
typedef struct __attribute__((__packed__)) {
  int32_t  target_position;    ///< 0x607A
  uint32_t digital_outputs;    ///< 0x60FE
  uint16_t controlword;        ///< 0x6040
  int32_t  target_velocity;    ///< 0x60FF
  int16_t  target_torque;      ///< 0x6071
  int32_t  position_offset;    ///< 0x60B0
  int32_t  velocity_offset;    ///< 0x60B1
  int16_t  torque_offset;      ///< 0x60B2
  int8_t   mode_of_operation;  ///< 0x6060
  uint16_t max_current;        ///< 0x6073 PL[1]
} jsd_egd_rxpdo_data_cs_mode_t;

/**
 * @brief RxPDO struct used to set device command data in SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 * See G-ETHERCATAM Section 6.1
 */
typedef struct __attribute__((__packed__)) {
  int32_t  target_position;    ///< 0x607A
  int32_t  target_velocity;    ///< 0x60FF
  uint16_t max_current;        ///< 0x6072 PL[1]
  uint16_t controlword;        ///< 0x6040
  int16_t  target_torque;      ///< 0x6071
  uint32_t profile_velocity;   ///< 0x6081
  uint32_t profile_accel;      ///< 0x6083
  uint32_t profile_decel;      ///< 0x6084
  uint32_t end_velocity;       ///< 0x6082
  int8_t   mode_of_operation;  ///< 0x6060
} jsd_egd_rxpdo_data_profiled_mode_t;

/**
 * @brief Elmo Gold Drive JSD Private State Data
 *
 * Any of the fields included here are for proper JSD function and not
 * necessarily helpful to the upstream application.
 */
typedef struct {
  jsd_egd_state_t pub;  ///< public state used by upstream applications

  // SOEM PDO data
  jsd_egd_txpdo_data_t               txpdo;       ///< Raw TxPDO data
  jsd_egd_rxpdo_data_cs_mode_t       rxpdo_cs;    ///< Raw RxPDO data
  jsd_egd_rxpdo_data_profiled_mode_t rxpdo_prof;  ///< Raw RxPDO data
  int8_t* mode_of_operation;  ///< ptr to active rxdpo mode of op

  uint32_t motor_rated_current;  ///< CL[1] in mA

  // user input
  bool                        new_reset;
  bool                        new_halt_command;
  bool                        new_motion_command;
  jsd_egd_motion_command_t    motion_command;  ///< Last command from user
  jsd_egd_mode_of_operation_t requested_mode_of_operation;
  double                      last_reset_time;

  // Fields parsed data from txpdo data
  uint8_t interlock;  ///< from DINs !(STO status) (firmware >= V1.1.10.7 B00)!
  uint8_t servo_enabled;               ///< from SR, of limited use
  uint8_t fault_occured_when_enabled;  ///< from SR, of limited use

  // Async SDO state
  double async_sdo_in_prog_start_time;

  // State tracking for smart printing
  jsd_egd_state_machine_state_t last_state_machine_state;
  jsd_egd_mode_of_operation_t   last_actual_mode_of_operation;
  jsd_egd_mode_of_operation_t   last_requested_mode_of_operation;
  uint8_t                       last_sto_engaged;
  bool                          last_async_sdo_in_prog;

} jsd_egd_private_state_t;

#ifdef __cplusplus
}
#endif

#endif
