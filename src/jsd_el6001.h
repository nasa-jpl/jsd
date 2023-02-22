#ifndef JSD_EL6001_H
#define JSD_EL6001_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_el6001_pub.h"

/**
 * @brief Initializes EL6001
 *
 * @param self Pointer to JSD context
 * @param slave_id Index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_el6001_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Read PDO data with SOEM IOmap
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 */
void jsd_el6001_read_PDO_data(jsd_t* self, uint16_t slave_id);

bool jsd_el6001_all_persistent_data_was_received(const jsd_t* self, uint16_t slave_id);

int jsd_el6001_set_transmit_data_8bits(jsd_t* self, uint16_t slave_id, int byte, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif