#ifndef JSD_EL2828_TYPES_H
#define JSD_EL2828_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL2828_PRODUCT_CODE (uint32_t)0x0B0C3052

#define JSD_EL2828_NUM_CHANNELS 8

/**
 * @brief EL2828 State Data
 */
typedef struct {
  uint8_t output[JSD_EL2828_NUM_CHANNELS];  ///< digital output level (0 or 1)
} jsd_el2828_state_t;

/**
 * @brief EL2828 device configuration
 */
typedef struct {
} jsd_el2828_config_t;

#ifdef __cplusplus
}
#endif

#endif
