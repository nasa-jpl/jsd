#ifndef JSD_EL1008_TYPES_H
#define JSD_EL1008_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL1008_PRODUCT_CODE (uint32_t)0x03f03052
#define JSD_EL1008_NUM_CHANNELS 8 // Each of the 8 values is 1 bit long.

/**
 * @brief configuration struct for EL1008 device initialization
 */
typedef struct {
} jsd_el1008_config_t;

/**
 * @brief Read struct for el1008 module
 */
typedef struct {
  uint8_t bitwise_values;
  bool values[JSD_EL1008_NUM_CHANNELS]; 
} jsd_el1008_state_t;

#ifdef __cplusplus
}
#endif

#endif
