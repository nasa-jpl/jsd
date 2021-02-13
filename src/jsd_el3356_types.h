#ifndef JSD_EL3356_TYPES_H
#define JSD_EL3356_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL3356_PRODUCT_CODE (uint32_t)0x0D1C3052

/**
 * @brief configuration struct for EL3356 device initialization
 *
 */
typedef struct {
  double scale_factor;  ///< User specified, used for state.scaled_value calc
} jsd_el3356_config_t;

/**
 * @brief Read struct for EL3356 module
 */
typedef struct {
  uint8_t overrange;     ///< measurement reached end value
  uint8_t data_invalid;  ///< display data is bad, possible bad calib.
  uint8_t error;         ///< True if any error is active
  uint8_t cal_in_prog;   ///< True if taring or performing other cal operation
  uint8_t steady_state;  ///< True when load is unchanging
  uint8_t sync_error;    ///< EtherCat Sync error active
  uint8_t txpdo_toggle;  ///< Toggles on new data
  int32_t value;         ///< Raw Sensed integer value, internal tare applied
  double
      scaled_value;  ///< Scaled by config.scale_facotr, internal tare applied

  uint8_t pending_tare;  ///< If user has requested tare, Internal Use mainly
} jsd_el3356_state_t;

#ifdef __cplusplus
}
#endif

#endif
