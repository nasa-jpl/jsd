#ifndef JSD_ILD1900_H
#define JSD_ILD1900_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd.h"

/**
 * @brief TxPDO struct used to read device data in SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  uint32_t timestamp;                  ///< 0x6002:1
  uint32_t counter;                    ///< 0X6003:1
  uint32_t sensor_status;              ///< 0x6004:1
  uint32_t unlinearized_distance_raw;  ///< 0x6005:1
  uint32_t intensity_raw;              ///< 0x6006:1
  uint32_t linearized_distance_raw;    ///< 0x6007:1
} jsd_ild1900_txpdo_t;

/**
 * @brief Initializes ILD1900 and registers the PO2SO function.
 *
 * @param self Pointer to JSD context
 * @param slave_id Index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_ild1900_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures ILD1900 device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id Index of device on EtherCAT bus
 * @return 1 on success, 0 on failure
 */
int jsd_ild1900_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

// Helper functions for PreOp to SafeOp configuration
bool jsd_ild1900_config_PDO_mapping(ecx_contextt* ecx_context,
                                    uint16_t      slave_id);
bool jsd_ild1900_config_COE_mapping(ecx_contextt*       ecx_context,
                                    uint16_t            slave_id,
                                    jsd_slave_config_t* config);

#ifdef __cplusplus
}
#endif

#endif