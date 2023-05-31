#ifndef JSD_EL3004_TYPES_H
#define JSD_EL3004_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL3004_PRODUCT_CODE (uint32_t)0x0BBC3052
#define JSD_EL3004_NUM_CHANNELS 4

/**
 * @brief Configuration struct for EL3004 device initialization
 */
typedef struct {
} jsd_el3004_config_t;

/**
 * @brief Read struct for EL3004 device
 */
typedef struct {
  double  voltage[JSD_EL3004_NUM_CHANNELS];    ///< Analog input data, converted
  int32_t adc_value[JSD_EL3004_NUM_CHANNELS];  ///< Analog input data, raw
  uint8_t underrange[JSD_EL3004_NUM_CHANNELS];  ///< True if value below
                                                ///< measuring range
  uint8_t
          overrange[JSD_EL3004_NUM_CHANNELS];  ///< True if measuring range exceeded
  uint8_t error[JSD_EL3004_NUM_CHANNELS];  ///< True if channel is over or under
                                           ///< range
} jsd_el3004_state_t;

#ifdef __cplusplus
}
#endif

#endif
