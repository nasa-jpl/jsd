#ifndef JSD_EL3104_H
#define JSD_EL3104_H

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
  int16_t  value;
} jsd_el3104_txpdo_channel_t;

/**
 * @brief TxPDO struct used to read device data in SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  jsd_el3104_txpdo_channel_t channel[JSD_EL3104_NUM_CHANNELS];
} jsd_el3104_txpdo_t;

///**
// * @brief Range Factor used for voltage calculation
// */
// static const double jsd_el3104_range_factor[] = {
//    [JSD_EL3104_RANGE_10V] = 20.0,  [JSD_EL3104_RANGE_5V] = 10.0,
//    [JSD_EL3104_RANGE_2_5V] = 5.0,  [JSD_EL3104_RANGE_1_25V] = 2.5,
//    [JSD_EL3104_RANGE_75MV] = 0.15, [JSD_EL3104_RANGE_200MV] = 0.4,
//};

/** @brief Initializes el3104 and registers the PO2SO function
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_el3104_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures EL3104 device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id index of device on EtherCAT bus
 * @return 1 on success, 0 on failure
 */
int jsd_el3104_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
