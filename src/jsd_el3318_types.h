#ifndef JSD_EL3318_TYPES_H
#define JSD_EL3318_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"
#include "jsd/jsd_include_defs.h"

#define JSD_EL3318_PRODUCT_CODE \
  (uint32_t)0x0cf63052  // This is for EL3318, 8-ch TC module
#define JSD_EL3318_NUM_CHANNELS 8

/**
 * @brief EL3318 Sensor Element Options
 */
typedef enum {
  JSD_EL3318_ELEMENT_TYPE_K = 0,  // -200 ~ 1370 C
  JSD_EL3318_ELEMENT_TYPE_J,      // -100 ~ 1200 C
  JSD_EL3318_ELEMENT_TYPE_L,      // 0 ~ 900 C
  JSD_EL3318_ELEMENT_TYPE_E,      // -100 ~ 1000 C
  JSD_EL3318_ELEMENT_TYPE_T,      // -200 ~ 400 C
  JSD_EL3318_ELEMENT_TYPE_N,      // -100 ~ 1300 C
  JSD_EL3318_ELEMENT_TYPE_U,      // 0 ~ 600 C
  JSD_EL3318_ELEMENT_TYPE_B,      // 600 ~  1800 C
  JSD_EL3318_ELEMENT_TYPE_R,      // 0 ~ 1767 C
  JSD_EL3318_ELEMENT_TYPE_S,      // 0 ~ 1760 C
  JSD_EL3318_ELEMENT_TYPE_C,      // 0 ~ 2320 C
  // Ask software dev for using voltage reading for EL3318
  JSD_EL3318_NUM_ELEMENTS,
} jsd_el3318_element_t;

extern const char* jsd_el3318_element_strings[];

typedef enum {
  JSD_EL3318_PRESENTATION_SIGNED =
      0,  ///< default 2-complement signed, 0.1 C/digit
  ////JSD_EL3318_PRESENTATION_MSB_SIGNED,  ///< No reason to use, unsupported
  JSD_EL3318_PRESENTATION_HIGH_RES = 2,  ///< 0.01 C/digit
  JSD_EL3318_NUM_PRESENTATIONS,
} jsd_el3318_presentation_t;

extern const char* jsd_el3318_presentation_strings[];

/**
 * @brief Config for EL3318 device
 */
typedef struct {
  jsd_el3318_element_t element[JSD_EL3318_NUM_CHANNELS];  ///< Type of TC
  jsd_beckhoff_filter_t
      filter[JSD_EL3208_NUM_CHANNELS];  ///< Digital Filter Opt
  jsd_el3318_presentation_t
      presentation[JSD_EL3318_NUM_CHANNELS];  ///< Data presentation, default:
                                              ///< low rez(1/10 C)
} jsd_el3318_config_t;

/**
 * @brief State for EL3318 device
 */
typedef struct {
  double output_eu[JSD_EL3318_NUM_CHANNELS];  ///< Converted Metric C or Ohms

  uint8_t underrange[JSD_EL3318_NUM_CHANNELS];  ///< Value below measuring range
  uint8_t overrange[JSD_EL3318_NUM_CHANNELS];   ///< Measuring range exceeded
  uint8_t error[JSD_EL3318_NUM_CHANNELS];       ///< if over or under range
  uint8_t txPDO_state[JSD_EL3318_NUM_CHANNELS];   ///< 0 - valid, 1 - invalid
  uint8_t txPDO_toggle[JSD_EL3318_NUM_CHANNELS];  ///< toggled on new data
  int16_t adc_value[JSD_EL3318_NUM_CHANNELS];     ///< sensed digital converted
                                                  ///< analog value

} jsd_el3318_state_t;

#ifdef __cplusplus
}
#endif

#endif
