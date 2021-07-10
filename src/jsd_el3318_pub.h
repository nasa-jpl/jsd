#ifndef JSD_EL3318_PUB_H
#define JSD_EL3318_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Reads EL3318 state
 *
 * @param self Pointer to JSD context
 * @param slave_id id of EL3318 device
 * @return Pointer to EL3318 state
 */
const jsd_el3318_state_t* jsd_el3318_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3318 device
 */
void jsd_el3318_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief Processes async SDO responses
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3318 device
 */
void jsd_el3318_process(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
