#ifndef JSD_EL1008_H
#define JSD_EL1008_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd.h"

/**
 * @brief Single channel of TxPDO data struct
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  int8_t values
} jsd_el1008_txpdo_channel_t;

/**
 * @brief TxPDO struct used to read device data in SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  jsd_el1008_txpdo_channel_t channel[1]; // A little hack as each input is 1 bit.
} jsd_el1008_txpdo_t;

/** @brief Initializes el1008 and registers the PO2SO function
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_el1008_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures EL1008 device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id index of device on EtherCAT bus
 * @return 1 on success, 0 on failure
 */
int jsd_el1008_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
