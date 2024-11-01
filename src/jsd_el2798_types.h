#ifndef JSD_EL2798_TYPES_H
#define JSD_EL2798_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL2798_PRODUCT_CODE (uint32_t)0x0AEE3052

#define JSD_EL2798_NUM_CHANNELS 8

/**
 * @brief EL2798 State Data
 */
typedef struct {
  uint8_t output[JSD_EL2798_NUM_CHANNELS];  ///< digital output level (0 or 1)
} jsd_el2798_state_t;

/**
 * @brief EL2798 device configuration
 */
typedef struct {
} jsd_el2798_config_t;

#ifdef __cplusplus
}
#endif

#endif
