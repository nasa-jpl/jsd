#ifndef JSD_EPD_H
#define JSD_EPD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_epd_pub.h"

// TODO(dloret): Consider making the data related to DS-402 common between the
// EGD and EPD drivers:
// - JSD_*_STATE_MACHINE_STATE_BITMASK
// - jsd_*_state_machine_controlword_t

/**
 * @brief Initializes EPD and registers the PO2SO function
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 * @return true on success, false on failure
 */
bool jsd_epd_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures EPD device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id Slave ID of EPD device
 * @return 1 on success, 0 on failure
 */
int jsd_epd_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

// Helper functions for jsd_epd_PO2SO_config
int jsd_epd_config_PDO_mapping(ecx_contextt* ecx_context, uint16_t slave_id);
int jsd_epd_config_COE_params(ecx_contextt* ecx_context, uint16_t slave_id,
                              jsd_slave_config_t* config);
int jsd_epd_config_LC_params(ecx_contextt* ecx_context, uint16_t slave_id,
                             jsd_slave_config_t* config);

// TODO(dloret): Removed helper function jsd_epd_read_PDO_data because it is
// effectively two lines. Unclear right now of whether a write helper is needed
// because I still do not know PDO byte limits.

/**
 * @brief Helper function of jsd_epd_read to extract state data from PDO data
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_update_state_from_PDO_data(jsd_t* self, uint16_t slave_id);

/**
 * @brief Helper function of jsd_epd_process to progress through the drive's
 * state machine
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_process_state_machine(jsd_t* self, uint16_t slave_id);

/**
 * @brief Helper function of jsd_epd_process to handle commands based on their
 * mode of operation.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_process_mode_of_operation(jsd_t* self, uint16_t slave_id);

// Helper functions to handle the different types of motion commands
void jsd_epd_mode_of_op_handle_csp(jsd_t* self, uint16_t slave_id);
void jsd_epd_mode_of_op_handle_csv(jsd_t* self, uint16_t slave_id);
void jsd_epd_mode_of_op_handle_cst(jsd_t* self, uint16_t slave_id);
void jsd_epd_mode_of_op_handle_prof_pos(jsd_t* self, uint16_t slave_id);
void jsd_epd_mode_of_op_handle_prof_vel(jsd_t* self, uint16_t slave_id);
void jsd_epd_mode_of_op_handle_prof_torque(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
