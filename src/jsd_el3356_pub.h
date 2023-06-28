#ifndef JSD_EL3356_PUB_H
#define JSD_EL3356_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_el3356_types.h"
#include "jsd/jsd_pub.h"

/**
 * @brief Read the EL3356 State
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3356 device
 * @return Pointer to EL3356 device state
 */
const jsd_el3356_state_t* jsd_el3356_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3356 device
 */
void jsd_el3356_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief process loop required for proper device function
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3356 device
 */
void jsd_el3356_process(jsd_t* self, uint16_t slave_id);

/**
 * @brief Tare the device
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL3356 device
 */
void jsd_el3356_tare(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
