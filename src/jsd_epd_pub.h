#ifndef JSD_EPD_PUB_H
#define JSD_EPD_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Reads the EPD device state
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @return Pointer to EPD device state
 */
const jsd_epd_state_t* jsd_epd_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief Processes state machine logic and copies commands in outgoing PDO
 *
 * Progresses through the state machine. Attempts to keep the drive status in
 * OPERATION_ENABLED if possible. Commands submitted through their corresponding
 * functions are effective in the cycle of the subsequent call to
 * jsd_epd_process.
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_process(jsd_t* self, uint16_t slave_id);

/**
 * @brief Resets the EPD after a fault
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_reset(jsd_t* self, uint16_t slave_id);

/**
 * @brief Halts the EPD. Requires reset to clear.
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_halt(jsd_t* self, uint16_t slave_id);

// TODO(dloret): add the following functions:
// - jsd_epd_set_gain_scheduling_index

/**
 * @brief Sets the value of the given digital output.
 *
 * Real-time safe.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param index Index of the digital output, [0, JSD_EPD_NUM_DIGITAL_OUTPUT)
 * @param output Output value (0 or 1)
 */
void jsd_epd_set_digital_output(jsd_t* self, uint16_t slave_id, uint8_t index,
                                uint8_t output);

/**
 * @brief Sets the peak current limit for the drive, P_PL[1]
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param peak_current Peak current limit to set in A
 */
void jsd_epd_set_peak_current(jsd_t* self, uint16_t slave_id,
                              double peak_current);

/**
 * @brief Sends a Cyclic Synchronous Position (CSP) motion command to the EPD
 * device
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param motion_command Set of parameters of the CSP command
 */
void jsd_epd_set_motion_command_csp(
    jsd_t* self, uint16_t slave_id,
    jsd_epd_motion_command_csp_t motion_command);

/**
 * @brief Sends a Cyclic Synchronous Velocity (CSV) motion command to the EPD
 * device
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param motion_command Set of parameters of the CSV command
 */
void jsd_epd_set_motion_command_csv(
    jsd_t* self, uint16_t slave_id,
    jsd_epd_motion_command_csv_t motion_command);

/**
 * @brief Sends a Cyclic Synchronous Torque (CST) motion command to the EPD
 * device.
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param motion_command Set of parameters of the CST command
 */
void jsd_epd_set_motion_command_cst(
    jsd_t* self, uint16_t slave_id,
    jsd_epd_motion_command_cst_t motion_command);

/**
 * @brief Sends a Profiled Position motion command to the EPD device.
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param motion_command Set of parameters of the Profiled Position command
 */
void jsd_epd_set_motion_command_prof_pos(
    jsd_t* self, uint16_t slave_id,
    jsd_epd_motion_command_prof_pos_t motion_command);

/**
 * @brief Sends a Profiled Velocity motion command to the EPD device.
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param motion_command Set of parameters of the Profiled Velocity command
 */
void jsd_epd_set_motion_command_prof_vel(
    jsd_t* self, uint16_t slave_id,
    jsd_epd_motion_command_prof_vel_t motion_command);

/**
 * @brief Sends a Profiled Torque motion command to the EPD device.
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param motion_command Set of parameters of the Profiled Torque command
 */
void jsd_epd_set_motion_command_prof_torque(
    jsd_t* self, uint16_t slave_id,
    jsd_epd_motion_command_prof_torque_t motion_command);

// TODO(dloret): think about how to handle informational printing (e.g.
// jsd_*_mode_of_operation_to_string, jsd_*_state_machine_to_string,
// jsd_*_fault_code_to_string).

/**
 * @brief Converts Elmo letter command to the associated SDO index
 *
 * @param letter_command Elmo command in capital characters (e.g. "PX")
 * @return Data Object index. If the command is not found, 0x0000 is returned.
 */
uint16_t jsd_epd_lc_to_do(char letter_command[2]);

// TODO(dloret): Add SDO-based function controller gain scheduling mode.

/**
 * @brief Sets drive's actual position (PX[1])
 *
 * Real-time safe as long as SDO thread holds the mutex to retrieve the SDO for
 * a deterministic amount of time. SDO is sent asynchronously in SDO thread.
 *
 * It is strongly recommended the application checks the result of the SDO-set
 * operation. Use the response queue contained in the jsd_t context to check for
 * the response and use the request data fields (such as the slave_id or app_id)
 * to verify result and handle SDO-set failures.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param position Position to set as current position (encoder counts)
 * @param app_id Application-provided ID for response tracking
 */
void jsd_epd_async_sdo_set_drive_position(jsd_t* self, uint16_t slave_id,
                                          double position, uint16_t app_id);

/**
 * @brief Sets the drive's Unit Mode (UM[1])
 *
 * Real-time safe as long as SDO thread holds the mutex to retrieve the SDO for
 * a deterministic amount of time. SDO is sent asynchronously in SDO thread.
 *
 * It is strongly recommended the application checks the result of the SDO-set
 * operation. Use the response queue contained in the jsd_t context to check for
 * the response and use the request data fields (such as the slave_id or app_id)
 * to verify result and handle SDO-set failures.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param mode Unit Mode to set (0-7)
 * @param app_id Application-provided ID for response tracking
 */
void jsd_epd_async_sdo_set_unit_mode(jsd_t* self, uint16_t slave_id,
                                     int16_t mode, uint16_t app_id);

/**
 * @brief Converts jsd_epd_mode_of_operation label to string
 *
 * @return string representation of enumeration label
 */
const char* jsd_epd_state_machine_state_to_string(
    jsd_epd_state_machine_state_t state);

/**
 * @brief Converts jsd_epd_fault_code_t label to string
 *
 * @return string representation of enumeration label
 */
const char* jsd_epd_fault_code_to_string(jsd_epd_fault_code_t fault_code);

#ifdef __cplusplus
}
#endif

#endif