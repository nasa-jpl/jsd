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
 * @brief Clears the state's latched errors.
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_clear_errors(jsd_t* self, uint16_t slave_id);

/**
 * @brief Halts the EPD. Requires reset to clear.
 *
 * Real-time safe
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_halt(jsd_t* self, uint16_t slave_id);

// TODO(dloret): Platinum drive is currently not responding to the index set in
// 0x36E0 when the controller scheduling mode is manual (GS[2]=67 or GS[2]=68).
// Need to wait for Elmo Support's solution.
/**
 * @brief Set the gain scheduling index manually
 *
 * Real-time safe
 *
 * Sets the index of the gain/parameter set for the controller or one of the
 * filters via Dictionary Object 0x36E0. Two indexes can be selected
 * independently through the LSB or MSB byte of the object. The GS command
 * determines which controller or filter is assigned the gains/parameters
 * specified by the index. See Platinum's Command Reference, command P_GS.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param lsb_byte Whether the provided index is for the controller/filter
 * assigned to the LSB (true) or MSB (false) of 0x36E0.
 * @param gain_scheduling_index Index of the gain/parameter set. It can range
 * from 1-63, and each value corresponds to a particular set of gains for the
 * controller or parameters for a filter.
 */
void jsd_epd_set_gain_scheduling_index(jsd_t* self, uint16_t slave_id,
                                       bool     lsb_byte,
                                       uint16_t gain_scheduling_index);

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
    jsd_elmo_motion_command_csp_t motion_command);

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
    jsd_elmo_motion_command_csv_t motion_command);

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
    jsd_elmo_motion_command_cst_t motion_command);

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
    jsd_elmo_motion_command_prof_pos_t motion_command);

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
    jsd_elmo_motion_command_prof_vel_t motion_command);

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
    jsd_elmo_motion_command_prof_torque_t motion_command);

/**
 * @brief Sets the value of the specified data input entry in SIL's R1 array.
 *
 * Real-time safe
 *
 * Entries R1[1] - R1[inputs_r1_num] are reserved for sending data to the drive
 * (i.e. inputs), where inputs_r1_num is the corresponding parameter of the Elmo
 * Platinum Drive JSD configuration. Given PDO channel's size limitations,
 * inputs_r1_num cannot exceed 32 and, in practice, will be less because of
 * other objects in the channel.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param subindex Subindex in the R1 array
 * @param value Value to set R1[subindex]
 */
void jsd_epd_set_sil_input_r1(jsd_t* self, uint16_t slave_id, uint16_t subindex,
                              int32_t value);

/**
 * @brief Sets the value of the specified data input entry in SIL's R2 array.
 *
 * Real-time safe
 *
 * Entries R2[1] - R2[inputs_r2_num] are reserved for sending data to the drive
 * (i.e. inputs), where inputs_r2_num is the corresponding parameter of the Elmo
 * Platinum Drive JSD configuration. Given PDO channel's size limitations,
 * inputs_r2_num cannot exceed 16 and, in practice, will be less because of
 * other objects in the channel.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param subindex Subindex in the R1 array
 * @param value Value to set R2[subindex]
 */
void jsd_epd_set_sil_input_r2(jsd_t* self, uint16_t slave_id, uint16_t subindex,
                              double value);

/**
 * @brief Gets the value of the specified data input entry in SIL's R1 array.
 *
 * Real-time safe
 *
 * Entries R1[1] - R1[inputs_r1_num] are reserved for sending data to the drive
 * (i.e. inputs), where inputs_r1_num is the corresponding parameter of the Elmo
 * Platinum Drive JSD configuration. Given PDO channel's size limitations,
 * inputs_r1_num cannot exceed 32 and, in practice, will be less because of
 * other objects in the channel.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param subindex Subindex in the R1 array
 * @return Value of R1[subindex]
 */
int32_t jsd_epd_get_sil_input_r1(jsd_t* self, uint16_t slave_id,
                                 uint16_t subindex);

/**
 * @brief Gets the value of the specified data input entry in SIL's R2 array.
 *
 * Real-time safe
 *
 * Entries R2[1] - R2[inputs_r2_num] are reserved for sending data to the drive
 * (i.e. inputs), where inputs_r2_num is the corresponding parameter of the Elmo
 * Platinum Drive JSD configuration. Given PDO channel's size limitations,
 * inputs_r2_num cannot exceed 16 and, in practice, will be less because of
 * other objects in the channel.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param subindex Subindex in the R2 array
 * @return Value of R2[subindex]
 */
double jsd_epd_get_sil_input_r2(jsd_t* self, uint16_t slave_id,
                                uint16_t subindex);

/**
 * @brief Gets the value of the specified data output entry in SIL's R1 array.
 *
 * Real-time safe
 *
 * Entries R1[129] - R1[128 + outputs_r1_num] are reserved for receiving data
 * from the drive (i.e. outputs), where outputs_r1_num is the corresponding
 * parameter of the Elmo Platinum Drive JSD configuration. Given PDO channel's
 * size limitations, outputs_r1_num cannot exceed 32 and, in practice, will be
 * less because of other objects in the channel.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param subindex Subindex in the R1 array
 * @return Value of R1[subindex]
 */
int32_t jsd_epd_get_sil_output_r1(jsd_t* self, uint16_t slave_id,
                                  uint16_t subindex);

/**
 * @brief Gets the value of the specified data output entry in SIL's R2 array.
 *
 * Real-time safe
 *
 * Entries R2[65] - R2[64 + outputs_r2_num] are reserved for receiving data from
 * the drive (i.e. outputs), where outputs_r2_num is the corresponding parameter
 * of the Elmo Platinum Drive JSD configuration. Given PDO channel's size
 * limitations, outputs_r2_num cannot exceed 16 and, in practice, will be less
 * because of other objects in the channel.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param subindex Subindex in the R2 array
 * @return Value of R2[subindex]
 */
double jsd_epd_get_sil_output_r2(jsd_t* self, uint16_t slave_id,
                                 uint16_t subindex);

/**
 * @brief Sets the value of the specified entry in SIL's R1 array.
 *
 * Entries R1[1 + inputs_r1_num] - R1[128] and R1[129 + outputs_r1_num] -
 * R1[256] can be set via SDO, where inputs_r1_num and outputs_r1_num are the
 * corresponding parameters of the Elmo Platinum Drive JSD configuration .
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
 * @param subindex Subindex in the R1 array
 * @param value Value to set R1[subindex]
 * @param app_id Application-provided ID for response tracking
 */
void jsd_epd_async_sdo_set_sil_r1(jsd_t* self, uint16_t slave_id,
                                  uint16_t subindex, int32_t value,
                                  uint16_t app_id);

/**
 * @brief Sets the value of the specified entry in SIL's R1 array.
 *
 * Entries R2[1 + inputs_r2_num] - R2[64] and R2[65 + outputs_r2_num] - R2[128]
 * can be set via SDO, where inputs_r2_num and outputs_r2_num are the
 * corresponding parameters of the Elmo Platinum Drive JSD configuration .
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
 * @param subindex Subindex in the R2 array
 * @param value Value to set R2[subindex]
 * @param app_id Application-provided ID for response tracking
 */
void jsd_epd_async_sdo_set_sil_r2(jsd_t* self, uint16_t slave_id,
                                  uint16_t subindex, double value,
                                  uint16_t app_id);

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
 * @brief Set the gain scheduling mode for the controller, GS[2]
 *
 * Real-time safe as long as SDO thread holds the mutex to retrieve the SDO for
 * a deterministic amount of time. SDO is sent asynchronously in SDO thread.
 *
 * It is strongly recommended the application checks the result of the SDO-set
 * operation. Use the response queue contained in the jsd_t context to check for
 * the response and use the request data fields (such as the slave_id or app_id)
 * to verify result and handle SDO-set failures.
 *
 * Subsequent application calls to update the index through
 * jsd_epd_set_gain_scheduling_index depend on the result of this SDO operation.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @param mode Gain scheduling mode
 * @param app_id Application-provided ID for response tracking
 */
void jsd_epd_async_sdo_set_ctrl_gain_scheduling_mode(
    jsd_t* self, uint16_t slave_id, jsd_elmo_gain_scheduling_mode_t mode,
    uint16_t app_id);

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