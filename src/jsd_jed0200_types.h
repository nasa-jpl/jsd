#ifndef JSD_JED0200_TYPES_H
#define JSD_JED0200_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_JED0200_PRODUCT_CODE (uint32_t)0x00001001

/**
 * @brief configuration struct for JED0200 device initialization
 */
typedef struct {
  uint16_t initial_cmd;  ///< Initial cmd value
} jsd_jed0200_config_t;

/**
 * @brief state data for JED0200 module
 */

typedef struct {
  uint16_t status;        ///< Status value placeholder
  uint32_t ticks;         ///< Ticks
  float    voltage_hv;    ///< High voltage
  float    voltage_lv;    ///< Low voltage
  float    voltage_12v;   ///< 12v voltage
  float    temp_ambient;  ///< Ambient temperature
  float    temp_actuator; ///< Actuator temperature
  float    humidity;      ///< Humidity
  float    pressure;      ///< Pressure
  uint16_t brake_current; ///< Brake current
  uint16_t brake_cc_val;  ///< Brake CC val
  uint16_t cmd;           ///< User specified cmd mode sent to JED0200
} jsd_jed0200_state_t;

#ifdef __cplusplus
}
#endif

#endif
