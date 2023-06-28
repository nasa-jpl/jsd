#ifndef JSD_EL4102_H
#define JSD_EL4102_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_el4102_pub.h"

/**
 * @brief Single channel of RxPDO command data struct
 */
typedef struct __attribute__((__packed__)) {
  int16_t value;
} jsd_el4102_rxpdo_channel_t;

/**
 * @brief RxPDO struct used to set device command data in SOEM IOmap
 *
 * Note: struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  jsd_el4102_rxpdo_channel_t channel[JSD_EL4102_NUM_CHANNELS];
} jsd_el4102_rxpdo_t;

/**
 * @brief Initializes EL4102
 *
 * @param self Pointer to JSD context
 * @param slave_id Index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_el4102_init(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif