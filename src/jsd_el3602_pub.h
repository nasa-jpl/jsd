#ifndef JSD_EL3602_PUB_H
#define JSD_EL3602_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Read the EL3602 device state
 *
 * @param self pointer to JSD context
 * @param slave_id slave id of EL3602 device
 * @return Pointer to EL3602 device state
 */
const jsd_el3602_state_t* jsd_el3602_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3602 device
 */
void jsd_el3602_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief Processes Async SDO responses
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3602 device
 */
void jsd_el3602_process(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
