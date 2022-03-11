#ifndef JSD_EL4102_TYPES_H
#define JSD_EL4102_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL4102_PRODUCT_CODE (uint32_t)0x10063052

#define JSD_EL4102_NUM_CHANNELS 2

/**
 * @brief Configuration struct for EL4102 device initialization
 */
typedef struct {
} jsd_el4102_config_t;

/**
 * @brief EL4102 state data
 */
typedef struct {
  int16_t dac_output[JSD_EL4102_NUM_CHANNELS];     ///< 0-10 V (0x0000-0x7FFF)
                                                   ///< analog output
  double voltage_output[JSD_EL4102_NUM_CHANNELS];  ///< Voltage equivalent of
                                                   ///< dac_output computed by
                                                   ///< this driver
} jsd_el4102_state_t;

#ifdef __cplusplus
}
#endif

#endif