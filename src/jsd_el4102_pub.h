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
 * @brief Sets the voltage (V) command for the given channel
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL4102 device
 * @param channel Device channel to command
 * @param output Commanded voltage. Provided value is clamped within [0-10].
 */
void jsd_el4102_write_single_channel(jsd_t* self, uint16_t slave_id,
                                     uint8_t channel, double output);

/**
 * @brief Sets the voltage (V) command for each channel
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL4102 device
 * @param output Commanded voltages. Provided values are clamped within [0-10].
 */
void jsd_el4102_write_all_channels(jsd_t* self, uint16_t slave_id,
                                   double output[JSD_EL4102_NUM_CHANNELS]);

#ifdef __cplusplus
}
#endif

#endif