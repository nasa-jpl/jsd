#ifndef JSD_EL2124_H
#define JSD_EL2124_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_el2124_pub.h"

/** @brief Initializes el2124
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_el2124_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Checks whether a product code is compatible with EL2124.
 *
 * @param product_code The product code to be checked
 * @return True if the product code is compatible, false otherwise.
 */
bool jsd_el2124_product_code_is_compatible(uint32_t product_code);

#ifdef __cplusplus
}
#endif

#endif
