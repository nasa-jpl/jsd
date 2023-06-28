#ifndef JSD_JED0101_H
#define JSD_JED0101_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_jed0101_pub.h"

/**
 * @brief TxPDO struct used to read device data in SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  uint16_t status;
  uint32_t w;
  uint32_t x;
  uint32_t y;
  uint32_t z;
} jsd_jed0101_txpdo_t;

/**
 * @brief RxPDO struct used to command the device via SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  uint16_t cmd;
} jsd_jed0101_rxpdo_t;

/** @brief Initializes jed0101 and registers the PO2SO function
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_jed0101_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures JED0101 device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id index of device on EtherCAT bus
 * @return 1 on success, 0 on failure
 */
int jsd_jed0101_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
