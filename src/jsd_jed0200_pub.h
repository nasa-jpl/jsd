#ifndef JSD_JED0200_PUB_H
#define JSD_JED0200_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Read the JED0200 state
 *
 * @param self pointer to JSD context
 * @param slave_id slave id of JED0200
 * @return Pointer to JED0200 state
 */
const jsd_jed0200_state_t* jsd_jed0200_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Sets the cmd PDO register on the JED0200 board
 *
 * @param self pointer to JSD context
 * @param slave_id id of JED0200
 * @param cmd Command integer value
 */
void jsd_jed0200_set_cmd_value(jsd_t* self, uint16_t slave_id, uint16_t cmd);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id id of JED0200
 */
void jsd_jed0200_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief Processes Async SDO responses
 *
 * @param self pointer to JSD context
 * @param slave_id id of JED0200
 */
void jsd_jed0200_process(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
