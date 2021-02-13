#ifndef JSD_EL3208_TYPES_H
#define JSD_EL3208_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"
#include "jsd/jsd_include_defs.h"

#define JSD_EL3208_PRODUCT_CODE (uint32_t)0x0c883052
#define JSD_EL3208_NUM_CHANNELS 8

/**
 * @brief EL3208 Sensor Element Options
 */
typedef enum {
  JSD_EL3208_ELEMENT_PT100 = 0,
  JSD_EL3208_ELEMENT_NI100,
  JSD_EL3208_ELEMENT_PT1000,
  JSD_EL3208_ELEMENT_PT500,
  JSD_EL3208_ELEMENT_PT200,
  JSD_EL3208_ELEMENT_NI1000,
  JSD_EL3208_ELEMENT_NI1000_TK1500,
  JSD_EL3208_ELEMENT_NI120,
  JSD_EL3208_ELEMENT_OHMS4096,
  JSD_EL3208_ELEMENT_OHMS1024,
  JSD_EL3208_ELEMENT_KT100_ET_AL,
  JSD_EL3208_NUM_ELEMENTS,
} jsd_el3208_element_t;

extern const char* jsd_el3208_element_strings[];

typedef enum {
  JSD_EL3208_CONNECTION_2WIRE = 0,
  JSD_EL3208_CONNECTION_3WIRE,
  JSD_EL3208_CONNECTION_4WIRE,
  JSD_EL3208_CONNECTION_NOT_CONNECTED,
  JSD_EL3208_NUM_CONNECTIONS,
} jsd_el3208_connection_t;

extern const char* jsd_el3208_connection_strings[];

typedef enum {
  JSD_EL3208_PRESENTATION_SIGNED = 0,  ///< default 2-complement signed
  ////JSD_EL3208_PRESENTATION_MSB_SIGNED,  ///< No reason to use, unsupported
  JSD_EL3208_PRESENTATION_HIGH_RES = 2,  ///< For EL320x-00x0 devices only
  JSD_EL3208_NUM_PRESENTATIONS,
} jsd_el3208_presentation_t;

extern const char* jsd_el3208_presentation_strings[];

/**
 * @brief Config for EL3208 device
 */
typedef struct {
  jsd_el3208_element_t element[JSD_EL3208_NUM_CHANNELS];  ///< Type of RTD
  jsd_beckhoff_filter_t
      filter[JSD_EL3208_NUM_CHANNELS];  ///< Digital Filter Opt
  jsd_el3208_connection_t
         connection[JSD_EL3208_NUM_CHANNELS];       ///< 2,3, or 4 wire
  double wire_resistance[JSD_EL3208_NUM_CHANNELS];  ///< User supplied, in Ohms
  jsd_el3208_presentation_t
      presentation[JSD_EL3208_NUM_CHANNELS];  ///< Data presentation
} jsd_el3208_config_t;

/**
 * @brief State for EL3208 device
 */
typedef struct {
  double  output_eu[JSD_EL3208_NUM_CHANNELS];    ///< Converted Metric C or Ohms
  int16_t adc_value[JSD_EL3208_NUM_CHANNELS];    ///< sensed integer value
  uint8_t txPDO_state[JSD_EL3208_NUM_CHANNELS];  ///< 0 - valid, 1 - invalid
  uint8_t txPDO_toggle[JSD_EL3208_NUM_CHANNELS];  ///< toggled on new data
  uint8_t error[JSD_EL3208_NUM_CHANNELS];         ///< if over or under range
  uint8_t underrange[JSD_EL3208_NUM_CHANNELS];  ///< Value below measuring range
  uint8_t overrange[JSD_EL3208_NUM_CHANNELS];   ///< Measuring range exceeded

} jsd_el3208_state_t;

#ifdef __cplusplus
}
#endif

#endif
