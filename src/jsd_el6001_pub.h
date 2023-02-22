#ifndef JSD_EL6001_PUB_H
#define JSD_EL6001_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_pub.h"
#include "jsd/jsd_el6001_types.h"

/**
 * @brief Read the EL6001 device state
 *
 * Note: this device does not provide PDO feedback on state. This function reads
 * back the command sent to the EL6001 device.
 *
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL6001 device
 * @return Pointer to EL6001 device state
 */
const jsd_el6001_state_t* jsd_el6001_get_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Writes the set commands into the SOEM IOmap and processes asynchronous
 * SDO responses
 * Reads Process Data Objects
 * Writes control word to transition through states
 * Writes control word to transmit and receive data
 * After this function (in between jsd_process calls:
 * User can affect control word number of output bytes
 * User can affect transmit request bit
 * 
 * @param self Pointer to JSD context
 * @param slave_id Slave ID of EL6001 device
 */
void jsd_el6001_process(jsd_t* self, uint16_t slave_id);



#ifdef __cplusplus
}
#endif

#endif