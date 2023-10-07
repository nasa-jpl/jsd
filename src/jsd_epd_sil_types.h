#ifndef JSD_EPD_SIL_TYPES_H
#define JSD_EPD_SIL_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"
#include "jsd/jsd_elmo_common_types.h"
#include "jsd/jsd_epd_common_types.h"

// Maximum number of SIL R1/R2 variables that can be mapped to RxPDO or TxPDO.
#define JSD_EPD_SIL_R1_MAX_NUM 32
#define JSD_EPD_SIL_R2_MAX_NUM 16

/**
 * @brief Elmo Platinum Drive in SIL mode JSD Configuration
 */
typedef struct {
  // TODO(dloret): Verify that selected integer is appropriate for the type
  // specified in command reference. Parameters set via SDO

  // Parameters set via Two-Letter-Command (TLC)
  // IMPORTANT: Elmo alias objects are different between Gold and Platinum
  // lines.
  float peak_current_limit;             ///< P_PL[1] A
  float peak_current_time;              ///< P_PL[2] s
  float continuous_current_limit;       ///< P_CL[1] A, continuous current limit
  float motor_stuck_current_level_pct;  ///< P_CL[2] pct[0,100], < 2 disables
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

  uint8_t sil_r1_inputs_num;  ///< Number of R1 user parameters (32-bit integer)
                              ///< to send data to the drive
  uint8_t sil_r2_inputs_num;  ///< Number of R2 user parameters (64-bit floating
                              ///< point) to send data to the drive
  uint8_t sil_r1_outputs_num;  ///< Number of R1 user parameters (32-bit
                               ///< integer) to retrieve data from the drive
  uint8_t sil_r2_outputs_num;  ///< Number of R2 user parameters (64-bit
                               ///< floating) to retrieve data from the drive

  // Optional function pointer to be called during the EtherCAT state machine
  // transition from Pre-Operational to Safe-Operational to perform
  // user-specific configuration. Set to NULL if not used
  int (*PO2SO_config_user_callback)(ecx_contextt* ecx_context,
                                    uint16_t slave_id, void* user_data);
  // Optional pointer to user-provided structure to be passed to
  // PO2SO_config_user_callback. Set to NULL if not used.
  void* PO2SO_config_user_data;
} jsd_epd_sil_config_t;

/**
 * @brief Elmo Platinum Drive in SIL mode JSD State Data
 */
typedef struct {
  int32_t actual_position;  ///< Actual position, cnt
  int32_t actual_velocity;  ///< Actual velocity, cnt/s
  double  actual_current;   ///< Actual motor current, A

  double cmd_max_current;  ///< Limit on current command, A

  jsd_elmo_state_machine_state_t actual_state_machine_state;
  jsd_epd_mode_of_operation_t    actual_mode_of_operation;

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
  jsd_epd_fault_code_t fault_code;  ///< Fault based on Emergency error code.
  uint16_t             emcy_error_code;  ///< Emergency error codes.

  bool sil_initialized;  ///< Whether SIL is loaded and initialized.
  bool sil_running;      ///< Whether SIL is running.
  bool sil_faulted;      ///< Whether there is a SIL run time error.
} jsd_epd_sil_state_t;

/**
 * @brief TxPDO struct used to read device data mapped by default in SOEM IOmap
 *
 * Note: struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  uint16_t statusword;                 ///< 0x6041
  uint32_t status_register_1;          ///< 0x3607:01
  uint32_t status_register_2;          ///< 0x3607:02
  int8_t   mode_of_operation_display;  ///< 0x6061. TODO(dloret): Confirm with
                                     ///< Elmo whether mode of operation has any
                                     ///< effect in SIL.
  int32_t actual_position;        ///< 0x6064
  int32_t velocity_actual_value;  ///< 0x6069
  int16_t current_actual_value;   ///< 0x6078
} jsd_epd_sil_txpdo_data_t;

/**
 * @brief RxPDO struct used to set device command data mapped by default in SOEM
 * IOmap
 *
 * Note: struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  uint16_t controlword;  ///< 0x6040
  uint16_t max_current;  ///< 0x6073. TODO(dloret): Decide whether this variable
                         ///< should be kept as default.
  int8_t
      mode_of_operation;  ///< 0x6060. TODO(dloret): Confirm with Elmo whether
                          ///< mode of operation has any effect in SIL.
} jsd_epd_sil_rxpdo_data_t;

/**
 * @brief Elmo Platinum Drive JSD private state data
 *
 * The fields included here are for JSD's internal use and may not be helpful
 * for client applications.
 */
typedef struct {
  jsd_epd_sil_state_t pub;  ///< Public state used by client applications.

  // SOEM PDO data
  jsd_epd_sil_rxpdo_data_t default_rxpdo;  ///< Raw RxPDO data mapped by default
  jsd_epd_sil_txpdo_data_t default_txpdo;  ///< Raw TxPDO data mapped by default
  // Arrays to hold SIL variables mapped to RxPDO
  int32_t
      sil_r1_inputs[JSD_EPD_SIL_R1_MAX_NUM];  ///< 0x22F3 starting at subindex 1
  double
      sil_r2_inputs[JSD_EPD_SIL_R2_MAX_NUM];  ///< 0x22F4 starting at subindex 1
  // Arrays to hold SIL variables mapped to TxPDO
  int32_t sil_r1_outputs[JSD_EPD_SIL_R1_MAX_NUM];  ///< 0x22F3 starting at
                                                   ///< subindex 129
  double sil_r2_outputs[JSD_EPD_SIL_R2_MAX_NUM];   ///< 0x22F4 starting at
                                                   ///< subindex 65

  uint32_t motor_rated_current;  ///< Set the same as continuous current limit
                                 ///< (CL[1]), mA

  // User input
  bool                        new_reset;
  bool                        new_halt_command;
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
} jsd_epd_sil_private_state_t;

#ifdef __cplusplus
}
#endif

#endif
