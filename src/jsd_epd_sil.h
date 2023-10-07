#ifndef JSD_EPD_SIL_H
#define JSD_EPD_SIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_epd_sil_pub.h"

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
bool jsd_epd_sil_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures EPD device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id Slave ID of EPD device
 * @return 1 on success, 0 on failure
 */
int jsd_epd_sil_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

// Helper functions for jsd_epd_PO2SO_config
int jsd_epd_sil_config_PDO_mapping(ecx_contextt* ecx_context, uint16_t slave_id,
                                   jsd_slave_config_t* config);
int jsd_epd_sil_config_COE_params(ecx_contextt* ecx_context, uint16_t slave_id,
                                  jsd_slave_config_t* config);
int jsd_epd_sil_config_LC_params(ecx_contextt* ecx_context, uint16_t slave_id,
                                 jsd_slave_config_t* config);

/**
 * @brief Helper function for jsd_epd_config_PDO_mapping to perform the actual
 * PDO mapping for a PDO channel (i.e. RxPDO or TxPDO)
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id Slave of ID of EPD-SIL device
 * @param pdo_mapping_parameters Array of mapping parameter objects of the PDO
 * channel. For example, for rPDO it would be {0x1600, 0x1601, ...}, and for
 * tPDO it would be {0x1A00, 0x1A01, ...}.
 * @param pdo_default_mapping_elements Array of mapping record elements of
 * objects to be mapped by default in the PDO channel. The array entries must
 * follow the structure described in sections 5.5 and 5.6 of the Platinum
 * Administrative Guide. Each object has two entries in the array. The first
 * entry corresponds to the object's size and subindex. The second entry
 * corresponds to the object's index. Objects must be the ones referred in
 * jsd_epd_sil_txpdo_data_t or jsd_epd_rxpdo_data_t, in the same order. For
 * example, to map by default Statusword (0x6041) and Status Register 1
 * (0x3607:01) to the tPDO channel, pdo_default_mapping_elements would be
 * {0x0010, 0x6041, 0x0120, 0x3607}.
 * @param pdo_default_variables_num Number of application variables to map by
 * default
 * @param sil_r1_variables_num Number of SIL R1 variables to map to the PDO
 * channel (rPDO -> inputs, tPDO -> outputs)
 * @param sil_r2_variables_num Number of SIL R2 variables to map to the PDO
 * channel (rPDO -> inputs, tPDO -> outputs)
 * @param is_rxpdo Whether the mapping is for RxPDO (true) or TxPDO (false).
 * @return 1 on success, or 0 on failure.
 */
int jsd_epd_config_PDO_mapping_helper(ecx_contextt* ecx_context,
                                      uint16_t      slave_id,
                                      uint16_t      pdo_mapping_parameters[],
                                      uint16_t pdo_default_mapping_elements[],
                                      int      pdo_default_variables_num,
                                      int      sil_r1_variables_num,
                                      int sil_r2_variables_num, bool is_rxpdo);

// TODO(dloret): Removed helper function jsd_epd_read_PDO_data because it is
// effectively two lines. Unclear right now of whether a write helper is needed
// because I still do not know PDO byte limits.

/**
 * @brief Helper function of jsd_epd_read to extract state data from PDO data
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_sil_update_state_from_PDO_data(jsd_t* self, uint16_t slave_id);

/**
 * @brief Helper function of jsd_epd_process to progress through the drive's
 * state machine
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EPD device
 */
void jsd_epd_sil_process_state_machine(jsd_t* self, uint16_t slave_id);

/**
 * @brief Checks whether a product code is compatible with EPD-SIL driver
 *
 * @param product_code The product code to be checked
 * @return True if the product code is compatible, false otherwise.
 */
bool jsd_epd_sil_product_code_is_compatible(uint32_t product_code);

#ifdef __cplusplus
}
#endif

#endif
