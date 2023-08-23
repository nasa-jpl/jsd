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

/**
 * @brief Get PDO data and update state data 
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 */
void jsd_el6001_read(jsd_t* self, uint16_t slave_id);

/**
 * @brief Set a transmission buffer at specific byte index
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @param byte index of transmit buffer
 * @param value 1byte data to be populated
 */
int jsd_el6001_set_transmit_data_8bits(jsd_t* self, uint16_t slave_id, int byte, uint8_t value);

/**
 * @brief Set a transmission full buffer at specific byte index
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @param data_in pointer of array to be populated in transmit buffer
 * @param data_len length of data buffer to be populated
 */
int jsd_el6001_set_transmit_data_payload(jsd_t* self, uint16_t slave_id, uint8_t* data_in, uint8_t data_len);

int jsd_el6001_request_transmit_data(jsd_t* self, uint16_t slave_id, int num_bytes_to_transmit);

int jsd_el6001_set_persistent_transmit_data(jsd_t* self, uint16_t slave_id, bool is_persistent);
 
#ifdef __cplusplus
}
#endif

#endif