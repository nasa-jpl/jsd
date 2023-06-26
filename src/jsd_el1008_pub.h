#ifndef JSD_EL1008_PUB_H
#define JSD_EL1008_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Read the EL1008 device state
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL1008 device
 * @return Pointer to EL1008 device state
 */
const jsd_el1008_state_t* jsd_el1008_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id ID of EL1008 device
 */
void jsd_el1008_read(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif