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

// Helper function to check whether the product code of a device matches the
// product code specified by the user or a compatible code of the corresponding
// device-type family (e.g. Platinum's product codes).
bool jsd_product_codes_match(uint32_t user_product_code,
                             uint32_t device_product_code);

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
