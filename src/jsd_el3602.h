#ifndef JSD_EL3602_H
#define JSD_EL3602_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd.h"

/**
 * @brief Single channel of TxPDO data struct
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  uint16_t flags;
  int32_t  value;
} jsd_el3602_txpdo_channel_t;

/**
 * @brief TxPDO struct used to read device data in SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  jsd_el3602_txpdo_channel_t channel[JSD_EL3602_NUM_CHANNELS];
} jsd_el3602_txpdo_t;

/**
 * @brief Range Factor used for voltage calculation
 */
static const double jsd_el3602_range_factor[] = {
    [JSD_EL3602_RANGE_10V] = 20.0,  [JSD_EL3602_RANGE_5V] = 10.0,
    [JSD_EL3602_RANGE_2_5V] = 5.0,  [JSD_EL3602_RANGE_1_25V] = 2.5,
    [JSD_EL3602_RANGE_75MV] = 0.15, [JSD_EL3602_RANGE_200MV] = 0.4,
};

/** @brief Initializes el3602 and registers the PO2SO function
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_el3602_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures EL3602 device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id index of device on EtherCAT bus
 * @return 1 on success, 0 on failure
 */
int jsd_el3602_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
