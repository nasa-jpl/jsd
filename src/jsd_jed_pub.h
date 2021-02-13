#ifndef JSD_JED_PUB_H
#define JSD_JED_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Read the JED state
 *
 * @param self pointer to JSD context
 * @param slave_id slave id of JED
 * @return Pointer to JED state
 */
const jsd_jed_state_t* jsd_jed_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Sets the cmd PDO register on the JED board
 *
 * @param self pointer to JSD context
 * @param slave_id id of JED
 * @param cmd Command integer value
 */
void jsd_jed_set_cmd_value(jsd_t* self, uint16_t slave_id, uint16_t cmd);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id id of JED
 */
void jsd_jed_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief Processes Async SDO responses
 *
 * @param self pointer to JSD context
 * @param slave_id id of JED
 */
void jsd_jed_process(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
