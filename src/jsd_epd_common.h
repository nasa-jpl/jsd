#ifndef JSD_EPD_COMMON_H
#define JSD_EPD_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_elmo_common_types.h"
#include "jsd/jsd_epd_common_types.h"
#include "jsd/jsd_types.h"

#define JSD_EPD_STATE_MACHINE_STATE_BITMASK (uint16_t)0x6F

/**
 * @brief Bit patterns in the Controlword to trigger transitions in the EPD's
 * state machine
 *
 * Relevant bits in the Controlword are 0, 1, 2, 3, and 7. See Platinum
 * Communication and Interpreters section 4.7.37.
 */
typedef enum {
  JSD_EPD_STATE_MACHINE_CONTROLWORD_SHUTDOWN                   = 0x06,
  JSD_EPD_STATE_MACHINE_CONTROLWORD_SWITCH_ON                  = 0x07,
  JSD_EPD_STATE_MACHINE_CONTROLWORD_SWITCH_ON_ENABLE_OPERATION = 0x08,
  JSD_EPD_STATE_MACHINE_CONTROLWORD_DISABLE_VOLTAGE            = 0x00,
  JSD_EPD_STATE_MACHINE_CONTROLWORD_QUICK_STOP                 = 0x02,
  JSD_EPD_STATE_MACHINE_CONTROLWORD_DISABLE_OPERATION          = 0x07,
  JSD_EPD_STATE_MACHINE_CONTROLWORD_ENABLE_OPERATION           = 0x0F,
  JSD_EPD_STATE_MACHINE_CONTROLWORD_FAULT_RESET                = 0x80,
} jsd_epd_state_machine_controlword_t;

/**
 * @brief Converts Elmo letter command to the associated SDO index
 *
 * @param letter_command Elmo command in capital characters (e.g. "PX")
 * @return Data Object index. If the command is not found, 0x0000 is returned.
 */
uint16_t jsd_epd_lc_to_do_impl(char letter_command[2]);

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
void jsd_epd_async_sdo_set_drive_position_impl(jsd_t* self, uint16_t slave_id,
                                               double   position,
                                               uint16_t app_id);

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
void jsd_epd_async_sdo_set_unit_mode_impl(jsd_t* self, uint16_t slave_id,
                                          int16_t mode, uint16_t app_id);

/**
 * @brief Sets the gain scheduling mode for the controller, GS[2]
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
void jsd_epd_async_sdo_set_ctrl_gain_scheduling_mode_impl(
    jsd_t* self, uint16_t slave_id, jsd_elmo_gain_scheduling_mode_t mode,
    uint16_t app_id);

/**
 * @brief Converts jsd_epd_fault_code_t label to string
 *
 * @return string representation of enumeration label
 */
const char* jsd_epd_fault_code_to_string_impl(jsd_epd_fault_code_t fault_code);

/**
 * @brief Converts error code in SOEM's ec_errort to jsd_epd_fault_code_t label
 *
 * @param error SOEM's ec_errort instance
 * @return jsd_epd_fault_code_t enumeration label
 */
jsd_epd_fault_code_t jsd_epd_get_fault_code_from_ec_error(ec_errort error);

/**
 * @brief Checks whether a product code is compatible with EPD drivers
 *
 * @param product_code The product code to be checked
 * @return True if the product code is compatible, false otherwise.
 */
bool jsd_epd_product_code_is_compatible(uint32_t product_code);

#ifdef __cplusplus
}
#endif

#endif