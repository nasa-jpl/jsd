#ifndef JSD_ATI_FTS_H
#define JSD_ATI_FTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd.h"

/**
 * @brief RxPDO struct used to write data to SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  uint32_t control1;
  uint32_t control2;
} jsd_ati_fts_rxpdo_t;

/**
 * @brief TxPDO struct used to read device data in SOEM IOmap
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  int32_t  fx_counts;
  int32_t  fy_counts;
  int32_t  fz_counts;
  int32_t  tx_counts;
  int32_t  ty_counts;
  int32_t  tz_counts;
  uint32_t status_code;
  uint32_t sample_counter;
} jsd_ati_fts_txpdo_t;

/**
 * @brief Data Object struct used to read calibration block at 0x2040
 *
 * Note: Struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {
  char     serial_number[8];
  char     calibration_part_number[30];
  char     calibration_family[8];
  char     calibration_date[30];
  int32_t  calibration_matrix[36];
  uint8_t  force_units;
  uint8_t  torque_units;
  int32_t  max_axis_counts[6];
  int32_t  counts_per_force;
  int32_t  counts_per_torque;
  uint16_t pot_gain[6];
  uint16_t dac_offset[6];
} jsd_ati_fts_calibration_do_t;

/** @brief Initializes ati_fts and registers the PO2SO function
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_ati_fts_init(jsd_t* self, uint16_t slave_id);

/**
 * @brief Configuration function called by SOEM upon a PreOp to SafeOp state
 * transition that (re)configures ATI_FTS device settings
 *
 * @param ecx_context SOEM context pointer
 * @param slave_id index of device on EtherCAT bus
 * @return 1 on success, 0 on failure
 */
int jsd_ati_fts_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id);

/** @brief Parses status code and returns true if error is active
 *
 * The function will print a helpful error message if a slave error is present
 *
 * @param status_code Data Object 0x6010
 * @param slave_id index of device on EtherCAT bus
 * @return True on active fault
 */
bool jsd_ati_fts_parse_status_code(uint32_t status_code, uint16_t slave_id);

/** @brief Decodes int value to literal string
 *
 * Taken from ATI Document #9620-05-EtherCAT-05 Table 5.2
 *
 * @param force_units int input value
 * @return pointer to string literal
 */
const char* jsd_ati_fts_force_unit_to_string(uint8_t force_units);

/** @brief Decodes integer Torque Unit value to literal string
 *
 * Taken from ATI Document #9620-05-EtherCAT-05 Table 5.2
 *
 * @param torque_units integer input value
 * @return pointer to string literal
 */
const char* jsd_ati_fts_torque_unit_to_string(uint8_t torque_units);

#ifdef __cplusplus
}
#endif

#endif
