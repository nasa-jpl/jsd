#ifndef JSD_EL2809_TYPES_H
#define JSD_EL2809_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL2809_PRODUCT_CODE (uint32_t)0x0af93052

#define JSD_EL2809_NUM_CHANNELS 4

/**
 * @brief EL2809 State Data
 */
typedef struct {
  uint8_t output[JSD_EL2809_NUM_CHANNELS];  ///< digital output level (0 or 1)
} jsd_el2809_state_t;

/**
 * @brief EL2809 device configuration
 */
typedef struct {
} jsd_el2809_config_t;

#ifdef __cplusplus
}
#endif

#endif
