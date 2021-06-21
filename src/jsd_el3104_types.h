#ifndef JSD_EL3104_TYPES_H
#define JSD_EL3104_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL3104_PRODUCT_CODE (uint32_t)0x0c203052
#define JSD_EL3104_NUM_CHANNELS 4

#define JSD_EL3104_FILTER_400HZ 2


/**
 * @brief configuration struct for EL3104 device initialization
 *
 * The fastest beckhoff_filter_t is 30kHz digital filter but the sampling
 * conversion time of 3ms for 2-channel operation, this is the fastest the
 * module can sample.
 *
 * If you sample faster than the digital filter can update the device state,
 * you will recieve stale data. As a check, you can use the TxPDO toggle field
 * to see when the device has new data.
 *
 * Empirically tested with a signal generator, I am able to resolve a
 * ~187.5hz sine wave with filter setting BECKHOFF_FILTER_30000HZ
 */
typedef struct {
  uint16_t filter[JSD_EL3104_NUM_CHANNELS];  ///< Active filter
  bool   limit1_enable[JSD_EL3104_NUM_CHANNELS];          ///< enables limit1
  double limit1_voltage[JSD_EL3104_NUM_CHANNELS];         ///< limit 1 in Volts
  bool   limit2_enable[JSD_EL3104_NUM_CHANNELS];          ///< enables limit2
  double limit2_voltage[JSD_EL3104_NUM_CHANNELS];         ///< limit 2 in Volts
} jsd_el3104_config_t;

/**
 * @brief Read struct for el3104 module
 */
typedef struct {
  double  voltage[JSD_EL3104_NUM_CHANNELS];    ///< Analog input data, converted
  int16_t adc_value[JSD_EL3104_NUM_CHANNELS];  ///< Analog input data, raw
  uint8_t limit1[JSD_EL3104_NUM_CHANNELS];     ///< 0-off, 1-exceeded, 2-under
  uint8_t limit2[JSD_EL3104_NUM_CHANNELS];     ///< 0-off, 1-exceeded, 2-under
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
