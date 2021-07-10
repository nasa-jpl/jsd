#ifndef JSD_EL3202_PUB_H
#define JSD_EL3202_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Reads EL3202 state
 *
 * @param self Pointer to JSD context
 * @param slave_id id of EL3202 device
 * @return Pointer to EL3202 state
 */
const jsd_el3202_state_t* jsd_el3202_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3202 device
 */
void jsd_el3202_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief Processes async SDO responses
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3202 device
 */
void jsd_el3202_process(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
