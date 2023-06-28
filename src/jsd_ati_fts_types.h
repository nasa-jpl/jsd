#ifndef JSD_ATI_FTS_TYPES_H
#define JSD_ATI_FTS_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_ATI_FTS_PRODUCT_CODE (uint32_t)0x26483052

#define JSD_ATI_FTS_DEFAULT_WORD_CONTROL1 (uint32_t)0x0
#define JSD_ATI_FTS_DEFAULT_WORD_CONTROL2 (uint32_t)0x0

#define JSD_ATI_FTS_MAX_CALIBRATION_VALUE (uint32_t)15  // 2^4-1 = 16

/**
 * @brief configuration struct for ATI FTS device initialization
 *
 * Note: The following sensor conifguration properties are not exposed:
 *   - Transformation parameters - leave as default CAD sensing frame
 *   - Monitoring Conditions - Ignored
 *   - Filter Selection - not using any onboard filtering
 */
typedef struct {
  uint32_t calibration;  ///< used to select different onboard calibrations

  int32_t counts_per_force;   ///< reserved, read from slave during init
  int32_t counts_per_torque;  ///< reserved, read from slave during init

} jsd_ati_fts_config_t;

/**
 * @brief Read struct for ATI FTS module
 */
typedef struct {
  double   fx;              ///< Fx in EU (Engineering Units)
  double   fy;              ///< Fy in EU
  double   fz;              ///< Fz in EU
  double   tx;              ///< Tx in EU
  double   ty;              ///< Ty in EU
  double   tz;              ///< Tz in EU
  bool     active_error;    ///< True if status_code indicates error
  uint32_t status_code;     ///< bitfield of DO 0x6010
  uint32_t sample_counter;  ///< monotonically increases on new sample

} jsd_ati_fts_state_t;

#ifdef __cplusplus
}
#endif

#endif
