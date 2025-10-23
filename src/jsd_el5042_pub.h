#ifndef JSD_EL5042_PUB_H
#define JSD_EL5042_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Read the EL5042 device state
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL5042 device
 * @return Pointer to EL5042 device state
 */
const jsd_el5042_state_t* jsd_el5042_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id ID of EL5042 device
 */
void jsd_el5042_read(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
