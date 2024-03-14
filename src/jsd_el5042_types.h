#ifndef JSD_EL5042_TYPES_H
#define JSD_EL5042_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL5042_PRODUCT_CODE (uint32_t)0x13b23052
#define JSD_EL5042_NUM_CHANNELS 2

/**
 * @brief Configuration struct for EL5042 device initialization
 */
typedef struct {
} jsd_el5042_config_t;

/**
 * @brief Read struct for EL5042 device
 */
typedef struct {
  int64_t position[JSD_EL5042_NUM_CHANNELS];  ///< Position in counts
  uint8_t warning[JSD_EL5042_NUM_CHANNELS];
  uint8_t error[JSD_EL5042_NUM_CHANNELS];
  uint8_t ready[JSD_EL5042_NUM_CHANNELS];
  uint8_t diag[JSD_EL5042_NUM_CHANNELS];
  uint8_t txpdo_state[JSD_EL5042_NUM_CHANNELS];
  uint8_t input_cycle_counter[JSD_EL5042_NUM_CHANNELS];
} jsd_el5042_state_t;

#ifdef __cplusplus
}
#endif

#endif
