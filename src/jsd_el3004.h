#ifndef JSD_EL3004_H
#define JSD_EL3004_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd.h"

/**
 * @brief Single channel of TxPDO data struct
 *
 * Note: struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  uint8_t flags;
  int32_t value;
} jsd_el3004_txpdo_channel_t;

/**
 * @brief TxPDO struct used to read device data in SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  jsd_el3004_txpdo_channel_t channel[JSD_EL3004_NUM_CHANNELS];
} jsd_el3004_txpdo_t;

/**
 * @brief Intializes EL3004 and registers the PO2SO function
 *
 * @param self Pointer to JSD context
 * @param slave_id Index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_el3004_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures EL3004 device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id Index of device on EtherCAT bus
 * @return 1 on success, 0 on failure
 */
int jsd_el3004_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif