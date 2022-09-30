#include "jsd/jsd_ild1900.h"

#include <assert.h>

#include "jsd/jsd_sdo.h"

// Start of measuring range (SMR) for each ILD1900 device model in m
const double JSD_ILD1900_SMR_MAP[JSD_ILD1900_MODEL_NUM_LABELS] = {
    0.015, 0.02, 0.025, 0.04, 0.05, 0.06, 0.1, 0.015, 0.017, 0.02, 0.025, 0.04};

// Measuring range (MR) for each ILD1900 device model in m
const double JSD_ILD1900_MR_MAP[JSD_ILD1900_MODEL_NUM_LABELS] = {
    0.002, 0.01, 0.025, 0.05, 0.1, 0.2, 0.5, 0.002, 0.006, 0.01, 0.025, 0.05};

/****************************************************
 * Public functions
 ****************************************************/

const jsd_ild1900_state_t* jsd_ild1900_get_state(jsd_t*   self,
                                                 uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_ILD1900_PRODUCT_CODE);

  const jsd_ild1900_state_t* state = &self->slave_states[slave_id].ild1900;
  return state;
}

void jsd_ild1900_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_ILD1900_PRODUCT_CODE);

  jsd_ild1900_state_t* state = &self->slave_states[slave_id].ild1900;

  const jsd_ild1900_txpdo_t* txpdo =
      (jsd_ild1900_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  const jsd_ild1900_config_t* config = &self->slave_configs[slave_id].ild1900;

  state->timestamp_us            = txpdo->timestamp;
  state->counter                 = txpdo->counter;
  state->sensor_status           = txpdo->sensor_status;
  state->distance_raw            = txpdo->peak_distance;
  state->intensity               = (100 * txpdo->intensity_raw) / 1023.0;
  state->distance_m = ((int32_t)txpdo->peak_distance - 98232) / 65536.0 *
                          JSD_ILD1900_MR_MAP[config->model] +
                      JSD_ILD1900_SMR_MAP[config->model];

  // Determine whether there was an error with the measurement.
  switch (txpdo->linearized_distance_raw) {
    case 262076:
      state->error = JSD_ILD1900_ERROR_NO_PEAK;
      break;
    case 262077:
      state->error = JSD_ILD1900_ERROR_PEAK_BEFORE_MR;
      break;
    case 262078:
      state->error = JSD_ILD1900_ERROR_PEAK_AFTER_MR;
      break;
    case 262080:
      state->error = JSD_ILD1900_ERROR_CANNOT_EVALUATE_MEASUREMENT;
      break;
    case 262081:
      state->error = JSD_ILD1900_ERROR_PEAK_TOO_WIDE;
      break;
    case 262082:
      state->error = JSD_ILD1900_ERROR_LASER_OFF;
      break;
    default:
      state->error = JSD_ILD1900_ERROR_OK;
  }
}

void jsd_ild1900_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_ILD1900_PRODUCT_CODE);
  // no-op
}

/****************************************************
 * Private functions
 ****************************************************/

// Determines whether a given number is a power of 2.
bool is_power2(uint32_t n) {
  // If only one of the bits in the integer is set, it is a power of 2.
  uint32_t num_bits = 0;
  while (n) {
    num_bits += (n & 1);
    n >>= 1;
  }
  if (num_bits == 1) {
    return true;
  } else {
    return false;
  }
}

bool jsd_ild1900_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_ILD1900_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_MICROEPSILON_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_ild1900_PO2SO_config;

  return true;
}

int jsd_ild1900_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_ILD1900_PRODUCT_CODE);

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within the ecx_context and extract it here.
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;

  jsd_slave_config_t* config = &slave_configs[slave_id];

  // Reset all settings first. Object 0x3800, subindex 0x18 is a boolean
  // variable. When accessed as a single subindex, the bit is encoded in an
  // octet 0bxxxxxxxn (by EtherCAT standard).
  uint8_t reset = 1;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3800, 0x18,
                                  JSD_SDO_DATA_U8, &reset)) {
    return false;
  }

  if (!jsd_ild1900_config_PDO_mapping(ecx_context, slave_id)) {
    ERROR("Failed to map PDO parameters on ILD1900 slave %u", slave_id);
    return false;
  }

  if (!jsd_ild1900_config_COE_mapping(ecx_context, slave_id, config)) {
    ERROR("Failed to set COE parameters on ILD1900 slave %u", slave_id);
    return false;
  }

  config->PO2SO_success = true;
  SUCCESS("ILD1900 slave %u's parameters successfully configured and verified",
          slave_id);
  return true;
}

bool jsd_ild1900_config_PDO_mapping(ecx_contextt* ecx_context,
                                    uint16_t      slave_id) {
  MSG_DEBUG("Attempting to map custom ILD1900 PDO...");

  /*
   * TxPDO Mapping
   * 0x1A04 - Timestamp (0x6002:1)
   * 0x1A08 - Measurement counter (0x6003:1)
   * 0x1A0C - Status (0x6004:1)
   * 0x1A14 - Peak distance (0x6008:1)
   * 0x1A10 - Not linearized distance + intensity + distance (0x6005:1 +
   * 0x5006:1 + 0x6007:1)
   *
   * ILD1900 sensors do not support Complete Access. Steps to set TxPDO
   * assignment are the following:
   * 1. Write 0 to 0x1C13:0.
   * 2. Write the desired PDO mappings individually to their corresponding
   * subindices (greater than 0).
   * 3. Write the total number of PDO mappings to 0x1C13:0.
   */
  uint8_t num_entries = 0;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x1C13, 0x00,
                                  JSD_SDO_DATA_U8, &num_entries)) {
    return false;
  }
  uint16_t entry = 0x1A04;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x1C13, 0x01,
                                  JSD_SDO_DATA_U16, &entry)) {
    return false;
  }
  entry = 0x1A08;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x1C13, 0x02,
                                  JSD_SDO_DATA_U16, &entry)) {
    return false;
  }
  entry = 0x1A0C;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x1C13, 0x03,
                                  JSD_SDO_DATA_U16, &entry)) {
    return false;
  }
  entry = 0x1A14;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x1C13, 0x04,
                                  JSD_SDO_DATA_U16, &entry)) {
    return false;
  }
  entry = 0x1A10;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x1C13, 0x05,
                                  JSD_SDO_DATA_U16, &entry)) {
    return false;
  }
  num_entries = 5;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x1C13, 0x00,
                                  JSD_SDO_DATA_U8, &num_entries)) {
    return false;
  }

  return true;
}

bool jsd_ild1900_config_COE_mapping(ecx_contextt*       ecx_context,
                                    uint16_t            slave_id,
                                    jsd_slave_config_t* config) {
  // Set laser power to Full for active operation. See Operating Instructions
  // optoNCDT 1900-IE EtherCAT section 7.3.
  uint8_t laser_power = 1;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3000, 0x01,
                                  JSD_SDO_DATA_U8, (void*)&laser_power)) {
    return false;
  }

  // Set shutter mode to automatic to allow the selection through configuration
  // of the exposure mode.
  uint8_t shutter_mode = 1;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3200, 0x14,
                                  JSD_SDO_DATA_U8, (void*)&shutter_mode)) {
    return false;
  }

  // Disable second averaging function. First averaging function is selected
  // through configuration.
  uint8_t averaging_2_type = 0;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3400, 0x0B,
                                  JSD_SDO_DATA_U8, (void*)&averaging_2_type)) {
    return false;
  }

  // Turn off zeroing/mastering function.
  //  uint8_t mastering_set = 0;
  //  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3450, 0x04,
  //                                  JSD_SDO_DATA_U8, (void*)&mastering_set)) {
  //    if (ecx_iserror(ecx_context)) {
  //      ec_errort error;
  //      while (ecx_poperror(ecx_context, &error)) {
  //        if (ecx_context->slavelist[error.Slave].eep_id ==
  //        JSD_ILD1900_PRODUCT_CODE && error.Etype == EC_ERR_TYPE_SDO_ERROR) {
  //          MSG("SDO abort code %x", error.AbortCode);
  //        }
  //      }
  //    }
  //    return false;
  //  }
  //  MSG("Zeroing/Mastering Set/Reset: %u", mastering_set);
  //  float mastering_value = 0;
  //  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3450, 0x05,
  //                                  JSD_SDO_DATA_FLOAT,
  //                                  (void*)&mastering_value)) {
  //  if (ecx_iserror(ecx_context)) {
  //    ec_errort error;
  //    while (ecx_poperror(ecx_context, &error)) {
  //      if (ecx_context->slavelist[error.Slave].eep_id ==
  //      JSD_ILD1900_PRODUCT_CODE && error.Etype == EC_ERR_TYPE_SDO_ERROR) {
  //        MSG("SDO abort code %x", error.AbortCode);
  //      }
  //    }
  //  }
  //    return false;
  //  }
  //  MSG("Zeroing/Mastering value: %f", mastering_value);

  // Set the measuring rate.
  if (config->ild1900.measuring_rate > 10000.0) {
    ERROR("ILD1900 slave %u's measuring rate %lf must be less than 10,000 Hz.",
          slave_id, config->ild1900.measuring_rate);
    return false;
  }
  // Sensor expects kHz.
  float measuring_rate_khz = (float)(config->ild1900.measuring_rate / 1000.0);
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3200, 0x03,
                                  JSD_SDO_DATA_FLOAT,
                                  (void*)&measuring_rate_khz)) {
    return false;
  }

  // Set the exposure mode.
  uint8_t exposure_mode = config->ild1900.exposure_mode;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3200, 0x16,
                                  JSD_SDO_DATA_U8, (void*)&exposure_mode)) {
    return false;
  }

  // Set the peak selection strategy.
  uint8_t peak_selection = config->ild1900.peak_selection;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3200, 0x1E,
                                  JSD_SDO_DATA_U8, (void*)&peak_selection)) {
    return false;
  }

  // Set the type of the measurement averaging function.
  uint8_t averaging_1_type = config->ild1900.averaging_type;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3400, 0x01,
                                  JSD_SDO_DATA_U8, (void*)&averaging_1_type)) {
    return false;
  }

  // Set the number of measurements to average.
  uint32_t averaging_number = config->ild1900.averaging_number;
  switch (averaging_1_type) {
    case JSD_ILD1900_AVERAGING_NONE:
      break;
    case JSD_ILD1900_AVERAGING_MEDIAN:
      if (averaging_number == 3 || averaging_number == 5 ||
          averaging_number == 7 || averaging_number == 9) {
        if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3400, 0x03,
                                        JSD_SDO_DATA_U32,
                                        (void*)&averaging_number)) {
          return false;
        }
        break;
      } else {
        ERROR(
            "ILD1900 slave %u's averaging number %u must be 3, 5, 7, or 9 for "
            "median averaging.",
            slave_id, averaging_number);
        return false;
      }
    case JSD_ILD1900_AVERAGING_MOVING:
      if (averaging_number >= 2 && averaging_number <= 4096 &&
          is_power2(averaging_number)) {
        if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3400, 0x02,
                                        JSD_SDO_DATA_U32,
                                        (void*)&averaging_number)) {
          return false;
        }
        break;
      } else {
        ERROR(
            "ILD1900 slave %u's averaging number %u must be 2, 4, 8, ..., 4096 "
            "(powers of 2) for moving averaging.",
            slave_id, averaging_number);
        return false;
      }
    case JSD_ILD1900_AVERAGING_RECURSIVE:
      if (averaging_number >= 1 && averaging_number <= 32000) {
        if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3400, 0x04,
                                        JSD_SDO_DATA_U32,
                                        (void*)&averaging_number)) {
          return false;
        }
        break;
      } else {
        ERROR(
            "ILD1900 slave %u's averaging number %u must be in the range [1, "
            "32000] for recursive averaging.",
            slave_id, averaging_number);
        return false;
      }
    default:
      ERROR("ILD1900 slave %u's averaging type %u is invalid.", slave_id,
            averaging_1_type);
      return false;
  }

  return true;
}
