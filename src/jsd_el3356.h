#ifndef JSD_EL3356_H
#define JSD_EL3356_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_el3356_pub.h"

/**
 * @brief Private IOmap channel struct used to send device data via IOmap
 */
typedef struct __attribute__((__packed__)) {
  uint8_t cmd_fields;
  uint8_t padding;
} jsd_el3356_rxpdo_t;

/**
 * @brief Private IOmap struct used to retreive input data from SOEM IOmap
 */
typedef struct __attribute__((__packed__)) {
  uint16_t status_fields;
  int32_t  value;
} jsd_el3356_txpdo_t;

/** @brief Initializes el3356 and registers the PO2SO function
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_el3356_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures el3356 device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id index of device on EtherCAT bus
 * @return 1 on success, 0 on failure
 */
int jsd_el3356_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
