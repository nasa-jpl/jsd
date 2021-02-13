#ifndef JSD_EGD_PUB_H
#define JSD_EGD_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Read the EGD device state
 *
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id slave id of EGD device
 * @return Pointer to Elmo Gold Drive State
 */
const jsd_egd_state_t* jsd_egd_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Reset the EGD after a fault
 *
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id slave id of EGD device
 */
void jsd_egd_reset(jsd_t* self, uint16_t slave_id);

/**
 * @brief Halt the EGD, requires reset to clear
 *
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id slave id of EGD device
 */
void jsd_egd_halt(jsd_t* self, uint16_t slave_id);

/**
 * @brief Disables motor motion, requires reset to clear
 *
 * Real-time safe.
 * Functionally identical to a halt command
 *
 * @param self pointer JSD context
 * @param slave_id slave id of EGD device
 */
void jsd_egd_disable_drive(jsd_t* self, uint16_t slave_id);

/**
 * @brief Set the Digital output level
 *
 * Real-time safe.
 * Only avaiable to JSD_EGD_DRIVE_MODE_CMD_CS command mode
 *
 * Digital outputs mapped to a brake cannot be commanded through
 * this interface. See MAN-G-DS402 Section 5.3
 *
 * @param self pointer JSD context
 * @param slave_id slave id of EGD device
 * @param digital_output_index from zero to JSD_EGD_NUM_DIGITAL_OUTPUTS
 * @param output_level desired command level, should be 0 or 1
 */
void jsd_egd_set_digital_output(jsd_t* self, uint16_t slave_id,
                                uint8_t digital_output_index,
                                uint8_t output_level);

// Set Manual Gain TODO

/**
 * @brief Set drive peak current, PL[1]
 *
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param max_current new drive current in amps
 */
void jsd_egd_set_peak_current(jsd_t* self, uint16_t slave_id,
                              double max_current);

/**
 * @brief Send a Profiled Position motion command to Elmo Gold Drive
 *
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param motion_command struct that governs profiled position behavior
 */
void jsd_egd_set_motion_command_prof_pos(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_prof_pos_t motion_command);

/**
 * @brief Send a Profiled Velocity motion command to Elmo Gold Drive
 *
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param motion_command struct that governs profiled velocity behavior
 */
void jsd_egd_set_motion_command_prof_vel(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_prof_vel_t motion_command);

/**
 * @brief Send a Profiled Torque motion command to Elmo Gold Drive
 *
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param motion_command struct that governs profiled torque behavior
 */
void jsd_egd_set_motion_command_prof_torque(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_prof_torque_t motion_command);

/**
 * @brief Send a Cyclic Synchronous Position motion command to Elmo Gold Drive
 *
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param motion_command struct that governs CSP behavior
 */
void jsd_egd_set_motion_command_csp(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_csp_t motion_command);

/**
 * @brief Send a Cyclic Synchronous Velocity motion command to Elmo Gold Drive
 *
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param motion_command struct that governs CSV behavior
 */
void jsd_egd_set_motion_command_csv(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_csv_t motion_command);

/**
 * @brief Send a Cyclic Synchronous Torque motion command to Elmo Gold Drive
 *
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param motion_command struct that governs CST behavior
 */
void jsd_egd_set_motion_command_cst(
    jsd_t* self, uint16_t slave_id,
    jsd_egd_motion_command_cst_t motion_command);

/**
 * @brief Get PDO data and update state data
 *
 * egd_state_t for the slave_id will be updated after this function call
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 */
void jsd_egd_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief Process state machine logic and prepare PDO data
 *
 * Progresses through the state machines for user requested control mode.
 * Attempts to keep the drive status in OPERATION_ENABLED if possible.
 *
 * The user should call egd commands prior to calling
 * this function for minimal latency.
 * Real-time safe.
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 */
void jsd_egd_process(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts jsd_egd_mode_of_operation int to human-readable string
 *
 * Real-time safe.
 *
 * return jsd_egd_mode_of_operation as a string
 */
char* jsd_egd_mode_of_operation_to_string(jsd_egd_mode_of_operation_t mode);

/**
 * @brief Converts jsd_egd_state_machine_state_to_string int to human-readable
 * string
 *
 * Real-time safe.
 *
 * return jsd_egd_state_machine_state as a string
 */
char* jsd_egd_state_machine_state_to_string(
    jsd_egd_state_machine_state_t state);

/**
 * @brief Converts jsd_egd_fault_code int to human-readable string
 *
 * Real-time safe.
 *
 * return jsd_egd_fault_code as a string
 */
char* jsd_egd_fault_code_to_string(jsd_egd_fault_code_t fault_code);

/**
 * @brief Two-Letter Command to Data Object
 *
 * The Two-Letter Commands map to registers in the 0x3000 to 0x3FFF range.
 *
 * This function does not assert on bad parameters, only warns users via STDOUT.
 * The improper SDO request will propagate through JSD and likely result
 * in a latched SDO error that can be cleared by a reset;
 *
 * @param tlc Two-Letter Command in capital chars  e.g. "PX"
 * @return Data Object register
 */
uint16_t jsd_egd_tlc_to_do(char tlc[2]);

/**
 * @brief Set drive position in encoder counts, PX[1]
 *
 * Real-time safe, but uses SDO background thread.
 * See MAN-G-CR for more information.
 *
 * It is recommended the application checks the status
 * of the async_sdo_in_prog state field before issuing
 * commands that may depend on this setting.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param position new drive position in encoder counts
 */
void jsd_egd_async_sdo_set_drive_position(jsd_t* self, uint16_t slave_id,
                                          int32_t position);

/**
 * @brief Set unit mode, UM[1]
 *
 * Real-time safe, but uses SDO background thread.
 * See MAN-G-CR for more information.
 *
 * It is recommended the application checks the status
 * of the async_sdo_in_prog state field before issuing
 * commands that may depend on this setting.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param mode user specified drive unit mode in range (1,6)
 */
void jsd_egd_async_sdo_set_unit_mode(jsd_t* self, uint16_t slave_id,
                                     int32_t mode);

#ifdef __cplusplus
}
#endif

#endif
