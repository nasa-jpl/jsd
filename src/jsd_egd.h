#ifndef JSD_EGD_H
#define JSD_EGD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_egd_pub.h"

// TODO maybe make this an enum?
#define JSD_EGD_STATE_MACHINE_STATE_BITMASK (uint16_t)0x6F

/**
 * @brief Controlwords used to transition through the EGD State Machine
 *
 * Controlword bits 0, 1, 2, 3, and 7 are used to dictate state machine
 * transitions. See MAN-G-DS402 6.5
 */
typedef enum {
  JSD_EGD_STATE_MACHINE_CONTROLWORD_SHUTDOWN                   = 0x06,
  JSD_EGD_STATE_MACHINE_CONTROLWORD_SWITCH_ON                  = 0x07,
  JSD_EGD_STATE_MACHINE_CONTROLWORD_SWITCH_ON_ENABLE_OPERATION = 0x08,
  JSD_EGD_STATE_MACHINE_CONTROLWORD_DISABLE_VOLTAGE            = 0x00,
  JSD_EGD_STATE_MACHINE_CONTROLWORD_QUICK_STOP                 = 0x02,
  JSD_EGD_STATE_MACHINE_CONTROLWORD_DISABLE_OPERATION          = 0x07,
  JSD_EGD_STATE_MACHINE_CONTROLWORD_ENABLE_OPERATION           = 0x0F,
  JSD_EGD_STATE_MACHINE_CONTROLWORD_FAULT_RESET                = 0x80,
} jsd_egd_state_machine_controlword_t;

/**
 * @brief initialize egd device state and register PO2SO function
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_egd_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures EGD device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id index of device on EtherCAT bus
 * @return 1 on success, 0 on failure
 */
int jsd_egd_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

int jsd_egd_config_PDO_mapping(ecx_contextt* ecx_context, uint16_t slave_id,
                               jsd_slave_config_t* config);
int jsd_egd_config_COE_params(ecx_contextt* ecx_context, uint16_t slave_id,
                              jsd_slave_config_t* config);
int jsd_egd_config_TLC_params(ecx_contextt* ecx_context, uint16_t slave_id,
                              jsd_slave_config_t* config);

/**
 * @brief Read PDO data with SOEM IOmap
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 */
void jsd_egd_read_PDO_data(jsd_t* self, uint16_t slave_id);

/**
 * @brief Write PDO data with SOEM IOmap
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 */
void jsd_egd_write_PDO_data(jsd_t* self, uint16_t slave_id);

/**
 * @brief Read this slave's async SDO response queue
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 */
void jsd_egd_async_sdo_process(jsd_t* self, uint16_t slave_id);

/**
 * @brief Parses RxPDO data to more useful state fields
 *
 * Should be called after jsd_egd_exchange_PDO_data
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 */
void jsd_egd_update_state_from_PDO_data(jsd_t* self, uint16_t slave_id);

/**
 * @brief Progresses drive through the statemachine and sets initial controlword
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 */
void jsd_egd_process_state_machine(jsd_t* self, uint16_t slave_id);

/**
 * @brief Manages the mode-of-operation-specific manipulation of
 * controlword and statusword.
 *
 * Controlword bits 4,5,6, and 8 are mode-of-operation-specific, see MAN-G-DS402
 * Section 6.5.
 * Statusword bits 12 and 13 are mode-of-ooperation-specific, see MAN-G-DS402
 * Section 6.6.
 * MAN-6-DS402 Sections 12 - 18 detail these registers for each mode of
 * operation.
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 */
void jsd_egd_process_mode_of_operation(jsd_t* self, uint16_t slave_id);

void jsd_egd_mode_of_op_handle_prof_pos(jsd_t* self, uint16_t slave_id);
void jsd_egd_mode_of_op_handle_prof_vel(jsd_t* self, uint16_t slave_id);
void jsd_egd_mode_of_op_handle_prof_torque(jsd_t* self, uint16_t slave_id);
void jsd_egd_mode_of_op_handle_csp(jsd_t* self, uint16_t slave_id);
void jsd_egd_mode_of_op_handle_csv(jsd_t* self, uint16_t slave_id);
void jsd_egd_mode_of_op_handle_cst(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts raw MF[1] fault wordo to JSD fault code type
 *
 * return fault code type as jsd_egd_fault_code_t
 */
jsd_egd_fault_code_t jsd_egd_get_fault_code_from_ec_error(ec_errort error);

#ifdef __cplusplus
}
#endif

#endif
