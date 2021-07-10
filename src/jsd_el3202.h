#ifndef JSD_EL3202_H
#define JSD_EL3202_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_el3202_pub.h"

/**
 * @brief Private IOmap channel struct used to retreive input data from SOEM
 * IOmap
 */
typedef struct __attribute__((__packed__)) {
  uint16_t flags;
  int16_t  value;
} jsd_el3202_channel_txpdo_t;

/**
 * @brief Private IOmap struct used to retreive input data from SOEM IOmap
 */
typedef struct __attribute__((__packed__)) {
  jsd_el3202_channel_txpdo_t channel[JSD_EL3202_NUM_CHANNELS];
} jsd_el3202_txpdo_t;

/** @brief Initializes el3202 and registers the PO2SO function
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_el3202_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures el3202 device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id index of device on EtherCAT bus
 * @return 1 on success, 0 on failure
 */
int jsd_el3202_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

/**
 * @brief Computes the state output_eu field using device config
 */
double jsd_el3202_output_from_config(int16_t              adc_value,
                                     jsd_el3202_config_t* config,
                                     uint16_t             channel);

#ifdef __cplusplus
}
#endif

#endif
