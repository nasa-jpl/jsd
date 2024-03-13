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
  double  voltage[JSD_EL5042_NUM_CHANNELS];    ///< Analog input data, converted
  int16_t adc_value[JSD_EL5042_NUM_CHANNELS];  ///< Analog input data, raw
  uint8_t underrange[JSD_EL5042_NUM_CHANNELS];  ///< True if value below
                                                ///< measuring range
  uint8_t
          overrange[JSD_EL5042_NUM_CHANNELS];  ///< True if measuring range exceeded
  uint8_t error[JSD_EL5042_NUM_CHANNELS];  ///< True if channel is over or under
                                           ///< range
} jsd_el5042_state_t;

#ifdef __cplusplus
}
#endif

#endif
