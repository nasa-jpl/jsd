#ifndef JSD_EL2798_PUB_H
#define JSD_EL2798_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_el2798_types.h"
#include "jsd/jsd_pub.h"

/**
 * @brief Read the EL2798 State
 *
 * Note: this device does not actually provide PDO feedback on state,
 * This function reads back the cmd sent to the EL2798 device
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL2798 device
 * @return Pointer to EL2798 device state
 */
const jsd_el2798_state_t* jsd_el2798_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief process loop required for proper device function
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL2798 device
 */
void jsd_el2798_process(jsd_t* self, uint16_t slave_id);

/**
 * @brief Sets a specified channel level
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL2798 device
 * @param channel specified device channel to command
 * @param output command level (0 or 1)
 */
void jsd_el2798_write_single_channel(jsd_t* self, uint16_t slave_id,
                                     uint8_t channel, uint8_t output);

/**
 * @brief Sets all channel levels
 *
 * @param self pointer to JSD context
 * @param slave_id id of EL2798 device
 * @param output command level (0 or 1)
 */
void jsd_el2798_write_all_channels(jsd_t* self, uint16_t slave_id,
                                   uint8_t output[JSD_EL2798_NUM_CHANNELS]);

#ifdef __cplusplus
}
#endif

#endif
