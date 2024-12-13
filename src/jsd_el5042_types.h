#ifndef JSD_EL5042_TYPES_H
#define JSD_EL5042_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL5042_PRODUCT_CODE (uint32_t)0x13b23052
#define JSD_EL5042_NUM_CHANNELS 2

/**
 * @brief EL5042 BiSS-C clock frequencies
 */
typedef enum {
  JSD_EL5042_10MHz = 0,
  JSD_EL5042_5MHz = 1,    
  JSD_EL5042_3_33MHz = 2,   
  JSD_EL5042_2_5MHz = 3,  
  JSD_EL5042_2MHz = 4,   
  JSD_EL5042_1MHz = 9,    
  JSD_EL5042_500KHz = 17,
  JSD_EL5042_250KHz = 19,
} jsd_el5042_clock_t;

extern const char* jsd_el5042_clock_strings[];

/**
 * @brief Configuration struct for EL5042 device initialization
 */
typedef struct {
  uint8_t invert_feedback_direction[JSD_EL5042_NUM_CHANNELS];
  uint8_t disable_status_bits[JSD_EL5042_NUM_CHANNELS];
  uint8_t invert_checksum[JSD_EL5042_NUM_CHANNELS];
  uint32_t checksum_polynomial[JSD_EL5042_NUM_CHANNELS];
  uint8_t supply_voltage[JSD_EL5042_NUM_CHANNELS];
  jsd_el5042_clock_t clock_frequency[JSD_EL5042_NUM_CHANNELS];
  uint8_t gray_code[JSD_EL5042_NUM_CHANNELS];
  uint8_t multiturn_bits[JSD_EL5042_NUM_CHANNELS];
  uint8_t singleturn_bits[JSD_EL5042_NUM_CHANNELS];
  uint8_t offset_bits[JSD_EL5042_NUM_CHANNELS];
  uint8_t ssi_mode[JSD_EL5042_NUM_CHANNELS];
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
