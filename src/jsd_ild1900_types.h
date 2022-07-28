#ifndef JSD_ILD1900_TYPES_H
#define JSD_ILD1900_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_ILD1900_PRODUCT_CODE (uint32_t)0x60CB01F6

/**
 * @brief ILD1900 device models
 *
 * Each model has a specific measuring range and a start of such range. The LL
 * models use a laser line especially suitable for shiny metallic, rough and
 * structured surfaces.
 */
typedef enum {
  JSD_ILD1900_MODEL_2 = 0,  ///< 2 mm measuring range
  JSD_ILD1900_MODEL_10,     ///< 10 mm measuring range
  JSD_ILD1900_MODEL_25,     ///< 25 mm measuring range
  JSD_ILD1900_MODEL_50,     ///< 50 mm measuring range
  JSD_ILD1900_MODEL_100,    ///< 100 mm measuring range
  JSD_ILD1900_MODEL_200,    ///< 200 mm measuring range
  JSD_ILD1900_MODEL_500,    ///< 500 mm measuring range
  JSD_ILD1900_MODEL_2LL,    ///< 2 mm measuring range
  JSD_ILD1900_MODEL_6LL,    ///< 6 mm measuring range
  JSD_ILD1900_MODEL_10LL,   ///< 10 mm measuring range
  JSD_ILD1900_MODEL_25LL,   ///< 25 mm measuring range
  JSD_ILD1900_MODEL_50LL,   ///< 50 mm measuring range
  JSD_ILD1900_MODEL_NUM_LABELS
} jsd_ild1900_model_t;

/**
 * @brief ILD1900's exposure mode
 *
 * See Operating Instructions optoNCDT 1900-IE EtherCAT section 7.4.5.
 */
typedef enum {
  JSD_ILD1900_EXPOSURE_MODE_STANDARD =
      0,  ///< Sensor adjusts exposure time so that intensity is 50%.
  JSD_ILD1900_EXPOSURE_MODE_INTELLIGENT =
      1,  ///< Ideal for moving objects or material transitions
  JSD_ILD1900_EXPOSURE_MODE_BACKGROUND =
      2  ///< Improves ambient light tolerance, but halves output rate.
} jsd_ild1900_exposure_mode_t;

/**
 * @brief ILD1900's peak selection
 *
 * Defines which signal in the array signal is used for evaluation.
 * See Operating Instructions optoNCDT 1900-IE EtherCAT section 7.4.6.
 */
typedef enum {
  JSD_ILD1900_PEAK_SELECTION_HIGHEST = 0,  ///< Peak with highest intensity
  JSD_ILD1900_PEAK_SELECTION_WIDEST  = 1,  ///< Peak with largest surface
  JSD_ILD1900_PEAK_SELECTION_LAST    = 2,  ///< Peak furthest away from sensor
  JSD_ILD1900_PEAK_SELECTION_FIRST   = 3   ///< Nearest peak to sensor
} jsd_ild1900_peak_selection_t;

/**
 * @brief ILD1900's type of averaging on measurements
 *
 * See Operating Instructions optoNCDT 1900-IE EtherCAT section 7.5.2.
 * The values of the averaging type objects (0x3400, subindeces 0x01 and 0x0B)
 * are incorrect in the documentation. The correct values are the ones assigned
 * in this enumeration.
 */
typedef enum {
  JSD_ILD1900_AVERAGING_NONE = 0,  ///< No averaging
  JSD_ILD1900_AVERAGING_MEDIAN =
      3,  ///< Median from specified number of measurements. Number must be 3,
          ///< 5, 7, or 9.
  JSD_ILD1900_AVERAGING_MOVING =
      1,  ///< Arithmetic average from the specified number of measurements.
          ///< Number must be a power of 2: 1, 2, 4, ..., 4096.
  JSD_ILD1900_AVERAGING_RECURSIVE =
      2  ///< Weighted average of new measured value with previous averaging
         ///< value. Number must be in range [1, 32000].
} jsd_ild1900_averaging_t;

/**
 * @brief Configuration struct for ILD1900 device initialization
 */
typedef struct {
  double measuring_rate;  ///< Number of measurements per second expressed in
                          ///< Hz. Maximum value is 10000 Hz. See Operating
                          ///< Instructions optoNCDT 1900-IE EtherCAT
                          ///< section 7.4.3.
  uint32_t
      averaging_number;  ///< Number of consecutive measurements averaged
                         ///< together. If averaging_type is
                         ///< JSD_ILD1900_AVERAGING_NONE, this value is ignored.
  jsd_ild1900_averaging_t
                               averaging_type;  ///< Type of averaging formula applied to measurements
  jsd_ild1900_model_t          model;  ///< Model number of the ILD1900 sensor
  jsd_ild1900_exposure_mode_t  exposure_mode;   ///< Exposure mode
  jsd_ild1900_peak_selection_t peak_selection;  ///< Peak selection strategy
} jsd_ild1900_config_t;

/**
 * @brief ILD1900's measurement errors
 */
typedef enum {
  JSD_ILD1900_ERROR_OK = 0,          ///< No error in the measurement
  JSD_ILD1900_ERROR_NO_PEAK,         ///< No peak found
  JSD_ILD1900_ERROR_PEAK_BEFORE_MR,  ///< Peak is before measuring range
  JSD_ILD1900_ERROR_PEAK_AFTER_MR,   ///< Peak is after measuring range
  JSD_ILD1900_ERROR_CANNOT_EVALUATE_MEASUREMENT,  ///< Measurement value cannot
                                                  ///< be evaluated
  JSD_ILD1900_ERROR_PEAK_TOO_WIDE,                ///< Peak is too wide
  JSD_ILD1900_ERROR_LASER_OFF                     ///< Laser is off
} jsd_ild1900_error_t;

/**
 * @brief Read struct for ILD1900 device
 */
typedef struct {
  double distance;  ///< Distance from the sensor to the target in m
  double
           unlinearized_center_of_gravity;  ///< Unlinearized distance in percentage
  double   intensity;      ///< Signal intensity of peak in percentage
  uint32_t timestamp;      ///< Timestamp of measurement in nanoseconds
  uint32_t
           counter;  ///< Measurement counter. Distinguishes separate measurements.
  uint32_t sensor_status;  ///< Bitfield with the status of the sensor's
                           ///< measurement. See Operating Instructions optoNCDT
                           ///< 1900-IE EtherCAT section 7.6.1.
  uint32_t peak_distance;  ///< Digital value of distance measured from SMR
  uint32_t linearized_distance_raw;  ///< Digital value of linearized distance
                                     ///< measured from SMR
  jsd_ild1900_error_t error;         ///< Type of measurement error
} jsd_ild1900_state_t;

#ifdef __cplusplus
}
#endif

#endif