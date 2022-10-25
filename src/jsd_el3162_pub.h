#ifndef JSD_EL3162_PUB_H
#define JSD_EL3162_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Read the EL3162 device state
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL3162 device
 * @return Pointer to EL3162 device state
 */
const jsd_el3162_state_t* jsd_el3162_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id ID of EL3162 device
 */
void jsd_el3162_read(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
