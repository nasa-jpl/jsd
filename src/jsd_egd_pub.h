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
 * @brief clears any latched state error fields
 *
 * Real-time safe. 
 * Does not reset drive to an operational state, drive will remain in 
 * SWITCHED_ON state until the reset is called.
 *
 * @param self pointer JSD context
 * @param slave_id slave id of EGD device
 */
void jsd_egd_clear_errors(jsd_t* self, uint16_t slave_id);

/**
 * @brief Reset the EGD after a fault and clear errors
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

/**
 * @brief Set the gain scheduling index manually
 *
 * Real-time safe
 * Only available to JSD_EGD_DRIVE_MODE_CMD_CS command mode
 *
 * Sets the index of the gain/parameter set for the controller or one of the
 * filters via Dictionary Object 0x2E00. Two indexes can be selected
 * independently through the LSB or MSB byte of the object. The GS command
 * determines which controller or filter is assigned the gains/parameters
 * specified by the index. See MAN-G-CR, command GS.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EGD device
 * @param lsb_byte Whether the provided index is for the controller/filter
 * assigned to the LSB (true) or MSB (false) of 0x2E00.
 * @param gain_scheduling_index Index of the gain/parameter set. It can range
 * from 1-63, and each value corresponds to a particular set of gains for the
 * controller or parameters for a filter.
 */
void jsd_egd_set_gain_scheduling_index(jsd_t* self, uint16_t slave_id,
                                       bool     lsb_byte,
                                       uint16_t gain_scheduling_index);

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
    jsd_elmo_motion_command_prof_pos_t motion_command);

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
    jsd_elmo_motion_command_prof_vel_t motion_command);

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
    jsd_elmo_motion_command_prof_torque_t motion_command);

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
    jsd_elmo_motion_command_csp_t motion_command);

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
    jsd_elmo_motion_command_csv_t motion_command);

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
    jsd_elmo_motion_command_cst_t motion_command);

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
uint16_t jsd_egd_tlc_to_do(const char tlc[2]);

/**
 * @brief Set drive position in encoder counts, PX[1]
 *
 * Real-time safe, but uses SDO background thread.
 * See MAN-G-CR for more information.
 *
 * It is strongly recommended the application checks the result of the SDO-set 
 * operation. Use the response queue contained in the jsd_t context to check for 
 * the response and use the request data fields (such as the slave_id or app_id)
 * to verify result and handle SDO-set failures.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param position new drive position in encoder counts
 * @param app_id application-provided id for response tracking, not touched by JSD
 * @return void
 */
void jsd_egd_async_sdo_set_drive_position(jsd_t* self, uint16_t slave_id,
                                          int32_t position, uint16_t app_id);

/**
 * @brief Set unit mode, UM[1]
 *
 * Real-time safe, but uses SDO background thread.
 * See MAN-G-CR for more information.
 *
 * It is strongly recommended the application checks the result of the SDO-set 
 * operation. Use the response queue contained in the jsd_t context to check for 
 * the response and use the request data fields (such as the slave_id or app_id)
 * to verify result and handle SDO-set failures.
 *
 * @param self pointer JSD context
 * @param slave_id id of Elmo Gold Drive on bus
 * @param mode user specified drive unit mode in range (1,6)
 * @param app_id application-provided id for response tracking, not touched by JSD
 * @return void
 */
void jsd_egd_async_sdo_set_unit_mode(jsd_t* self, uint16_t slave_id,
                                     int32_t mode, uint16_t app_id);

/**
 * @brief Set the gain scheduling mode for the controller, GS[2]
 *
 * Real-time safe, but uses SDO background thread.
 * See MAN-G-CR for more information.
 *
 * It is strongly recommended the application checks the result of the SDO-set 
 * operation. Use the response queue contained in the jsd_t context to check for 
 * the response and use the request data fields (such as the slave_id or app_id)
 * to verify result and handle SDO-set failures.
 *
 * Subsequent application calls to update the index through 
 * jsd_egd_set_gain_scheduling_index depends on the result of this SDO operation
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EGD device
 * @param mode Gain scheduling mode
 * @param app_id application-provided id for response tracking, not touched by JSD
 * @return void
 */
void jsd_egd_async_sdo_set_ctrl_gain_scheduling_mode(
    jsd_t* self, uint16_t slave_id, jsd_elmo_gain_scheduling_mode_t mode,
    uint16_t app_id);

#ifdef __cplusplus
}
#endif

#endif
