#ifndef JSD_EL3104_PUB_H
#define JSD_EL3104_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Read the EL3104 device state
 *
 * @param self pointer to JSD context
 * @param slave_id slave id of EL3104 device
 * @return Pointer to EL3104 device state
 */
const jsd_el3104_state_t* jsd_el3104_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3104 device
 */
void jsd_el3104_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief Processes Async SDO responses
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3104 device
 */
void jsd_el3104_process(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
