#ifndef JSD_ILD1900_PUB_H
#define JSD_ILD1900_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Read the ILD1900 device state
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of ILD1900 device
 * @param Pointer to ILD1900 device state
 */
const jsd_ild1900_state_t* jsd_ild1900_get_state(jsd_t*   self,
                                                 uint16_t slave_id);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of ILD1900 device
 */
void jsd_ild1900_read(jsd_t*, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
