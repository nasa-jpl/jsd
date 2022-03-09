#ifndef JSD_EL4102_PUB_H
#define JSD_EL4102_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_el4102_types.h"
#include "jsd/jsd_pub.h"

/**
 * @brief Read the EL4102 device state
 *
 * Note: this device does not provide PDO feedback on state. This function reads
 * back the command sent to the EL4102 device.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL4102 device
 * @return Pointer to EL4102 device state
 */
const jsd_el4102_state_t* jsd_el4102_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Writes the set commands into the SOEM IOmap and processes asynchronous
 * SDO responses
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL4102 device
 */
void jsd_el4102_process(jsd_t* self, uint16_t slave_id);

/**
 * @brief Sets the DAC value command for the given channel
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL4102 device
 * @param channel Device channel to command
 * @param output Commanded DAC value (0x0000-0x7FFF)
 */
void jsd_el4102_write_single_channel(jsd_t* self, uint16_t slave_id,
                                     uint8_t channel, int16_t output);

/**
 * @brief Sets the DAC value commands for each channel
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL4102 device
 * @param output Commanded DAC values (0x0000-0x7FFF)
 */
void jsd_el4102_write_all_channels(jsd_t* self, uint16_t slave_id,
                                   int16_t output[JSD_EL4102_NUM_CHANNELS]);

#ifdef __cplusplus
}
#endif

#endif