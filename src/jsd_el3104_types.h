#ifndef JSD_EL3104_TYPES_H
#define JSD_EL3104_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL3104_PRODUCT_CODE (uint32_t)0x0c203052
#define JSD_EL3104_NUM_CHANNELS 4

/**
 * @brief configuration struct for EL3104 device initialization
 */
typedef struct {
} jsd_el3104_config_t;

/**
 * @brief Read struct for el3104 module
 */
typedef struct {
  double  voltage[JSD_EL3104_NUM_CHANNELS];    ///< Analog input data, converted
  int16_t adc_value[JSD_EL3104_NUM_CHANNELS];  ///< Analog input data, raw
  uint8_t
          txPDO_state[JSD_EL3104_NUM_CHANNELS];  ///< 0 - data is valid, 1 otherwise
  uint8_t txPDO_toggle[JSD_EL3104_NUM_CHANNELS];  ///< toggled on new data
  uint8_t
      error[JSD_EL3104_NUM_CHANNELS];  ///< If channel is over or under range
  uint8_t
      sync_error[JSD_EL3104_NUM_CHANNELS];  /// True(DC mode) a synchronization
                                            /// error occurred in the expired
                                            /// cycle
  uint8_t
          underrange[JSD_EL3104_NUM_CHANNELS];  ///< Value below measuring range.
  uint8_t overrange[JSD_EL3104_NUM_CHANNELS];   ///< Measuring range exceeded.

} jsd_el3104_state_t;

#ifdef __cplusplus
}
#endif

#endif
