#ifndef JSD_H
#define JSD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

#define JSD_PO2OP_MAX_ATTEMPTS 3

/**
 * @brief converts ec_state int to human-readable string
 *
 * @return ec_state as a string
 */
char* jsd_ec_state_to_string(ec_state state);

/**
 * @brief Registers all device configs used by
 * SOEM during the EtherCat PreOp to SafeOp transition and any initial device
 * setup.
 *
 * @param self pointer JSD context
 * @return true if all device callbacks where registered correctly
 */
bool jsd_init_all_devices(jsd_t* self);

// Helper function to check whether a device type is compatible with the
// product code of an actual device.
bool jsd_device_is_compatible_with_product_code(jsd_device_type_t device_type,
                                                uint32_t          product_code);

/**
 * @brief Initializes single device
 *
 * Helper function for jsd_init_all_devices(...)
 *
 * @param self pointer JSD context
 * @param slave_id slave id of device
 * @return true if slave config was correctly registered
 */
bool jsd_init_single_device(jsd_t* self, uint16_t slave_id);

/**
 * @brief Monitors the state of slaves and attempts to recover from faults
 *
 * @param self pointer JSD context
 */
void jsd_ecatcheck(jsd_t* self);

#ifdef __cplusplus
}
#endif

#endif
