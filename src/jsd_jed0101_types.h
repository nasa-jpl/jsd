#ifndef JSD_JED0101_TYPES_H
#define JSD_JED0101_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_JED0101_PRODUCT_CODE (uint32_t)0x00009252

/**
 * @brief configuration struct for JED0101 device initialization
 */
typedef struct {
  uint16_t initial_cmd;  ///< Initial cmd value
} jsd_jed0101_config_t;

/**
 * @brief state data for JED0101 module
 */

typedef struct {
  uint16_t status;  ///< Status value placeholder
  uint32_t w_raw;   ///< raw w component of IMU quaternion
  uint32_t x_raw;   ///< raw x component of IMU quaternion
  uint32_t y_raw;   ///< raw y component of IMU quaternion
  uint32_t z_raw;   ///< raw z component of IMU quaternion
  double   w;       ///< converted w component of IMU quaternion
  double   x;       ///< converted x component of IMU quaternion
  double   y;       ///< converted y component of IMU quaternion
  double   z;       ///< converted z component of IMU quaternion
  uint16_t cmd;     ///< User specified cmd mode sent to JED0101
} jsd_jed0101_state_t;

#ifdef __cplusplus
}
#endif

#endif
