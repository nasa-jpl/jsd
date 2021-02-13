#ifndef JSD_COMMOMN_DEVICE_TYPES_H
#define JSD_COMMOMN_DEVICE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_include_defs.h"

#define JSD_BECKHOFF_VENDOR_ID (uint32_t)0x00000002

#define JSD_BECKHOFF_RESET_SDO (uint16_t)0x1011
#define JSD_BECKHOFF_RESET_SUBIND (uint8_t)0x01
#define JSD_BECKHOFF_RESET_WORD (uint32_t)0x64616F6C

#define JSD_ELMO_VENDOR_ID (uint32_t)0x0000009A

#define JSD_ATI_VENDOR_ID (uint32_t)0x00000732

#define JSD_JPL_VENDOR_ID (uint32_t)0x00000C53

/**
 * @brief Digital Filter Options common across many beckhoff devices
 */
typedef enum {
  JSD_BECKHOFF_FILTER_50HZ = 0,  ///< 50 Hz digital filter
  JSD_BECKHOFF_FILTER_60HZ,      ///< 60 Hz digital filter
  JSD_BECKHOFF_FILTER_100HZ,     ///< 100 Hz digital filter
  JSD_BECKHOFF_FILTER_500HZ,     ///< 500 Hz digital filter
  JSD_BECKHOFF_FILTER_1000HZ,    ///< 1000 Hz digital filter
  JSD_BECKHOFF_FILTER_2000HZ,    ///< 2000 Hz digital filter
  JSD_BECKHOFF_FILTER_3750HZ,    ///< 3750 Hz digital filter
  JSD_BECKHOFF_FILTER_7500HZ,    ///< 7500 Hz digital filter
  JSD_BECKHOFF_FILTER_15000HZ,   ///< 15,000 Hz digital filter
  JSD_BECKHOFF_FILTER_30000HZ,   ///< 30,000 Hz digital filter
  JSD_BECKHOFF_FILTER_5HZ,       ///< 5 Hz digital filter
  JSD_BECKHOFF_FILTER_10HZ,      ///< 10 Hz digital filter
  JSD_BECKHOFF_NUM_FILTERS,
} jsd_beckhoff_filter_t;

extern const char* jsd_beckhoff_filter_strings[];

#ifdef __cplusplus
}
#endif

#endif
