#ifndef JSD_ATI_FTS_PUB_H
#define JSD_ATI_FTS_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"

/**
 * @brief Read the ATI_FTS device state
 *
 * @param self pointer to JSD context
 * @param slave_id slave id of ATI_FTS device
 * @return Pointer to ATI_FTS device state
 */
const jsd_ati_fts_state_t* jsd_ati_fts_get_state(jsd_t*   self,
                                                 uint16_t slave_id);

/**
 * @brief Sets the active Calibration Integer
 *
 * Overrides the jsd_ati_fts_config_t specified calibration setting.
 * transmitted during each PDO frame. Should result in
 *
 * @param self pointer to JSD context
 * @param slave_id slave id of ATI_FTS device
 * @param calibration ATI Calibration Integer
 */
void jsd_ati_fts_set_calibration(jsd_t* self, uint16_t slave_id,
                                 uint32_t calibration);

/**
 * @brief Converts raw PDO data to state data
 *
 * @param self pointer to JSD context
 * @param slave_id id of ATI_FTS device
 */
void jsd_ati_fts_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief Processes Async SDO responses
 *
 * @param self pointer to JSD context
 * @param slave_id id of ATI_FTS device
 */
void jsd_ati_fts_process(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
