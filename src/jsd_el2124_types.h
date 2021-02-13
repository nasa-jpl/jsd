#ifndef JSD_EL2124_TYPES_H
#define JSD_EL2124_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL2124_PRODUCT_CODE (uint32_t)0x084c3052

#define JSD_EL2124_NUM_CHANNELS 4

/**
 * @brief EL2124 State Data
 */
typedef struct {
  uint8_t output[JSD_EL2124_NUM_CHANNELS];  ///< digital output level (0 or 1)
} jsd_el2124_state_t;

/**
 * @brief EL2124 device configuration
 */
typedef struct {
} jsd_el2124_config_t;

#ifdef __cplusplus
}
#endif

#endif
