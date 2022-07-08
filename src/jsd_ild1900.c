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

  // TODO(dloret): Confirm timestamp is in nanoseconds.
  state->timestamp               = txpdo->timestamp;
  state->sensor_status           = txpdo->sensor_status;
  state->linearized_distance_raw = txpdo->linearized_distance_raw;
  state->intensity               = (100 * txpdo->intensity_raw) / 1023.0;
  state->unlinearized_center_of_gravity =
      (100 * txpdo->unlinearized_distance_raw) / 262143.0;
  state->distance = (txpdo->linearized_distance_raw - 98232) / 65536.0 *
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

  // Iterate through this device's response queue in JSD's context, and check
  // for errors until all responses are popped.
  jsd_async_sdo_process_response(self, slave_id);
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

  // TODO(dloret): Issue SDO to reset all settings here (0x3800:18, bit, set to
  // 1).

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
   * 0x1A0C - Status (0x6004:1)
   * 0x1A10 - Not linearized distance + intensity + distance (0x6005:1 +
   * 0x5006:1 + 0x6007:1)
   */
  uint16_t map_input_TxPDO[] = {0x0003, 0x1A04, 0x1A0C, 0x1A10};

  if (!jsd_sdo_set_ca_param_blocking(ecx_context, slave_id, 0x1C13, 0x00,
                                     sizeof(map_input_TxPDO),
                                     &map_input_TxPDO)) {
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

  // Disable signal quality mode because averaging is selected through
  // configuration. See Operating Instructions optoNCDT 1900-IE EtherCAT
  // sections 6.2.2 and A5.3.2.7.
  uint8_t signal_quality_mode = 0;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3851, 0x01,
                                  JSD_SDO_DATA_U8,
                                  (void*)&signal_quality_mode)) {
    return false;
  }

  // Set the measuring rate.
  float measuring_rate_hz = config->ild1900.measuring_rate;
  if (measuring_rate_hz > 10000.0) {
    ERROR("ILD1900 slave %u's measuring rate %f must be less than 10,000 Hz.",
          slave_id, measuring_rate_hz);
    return false;
  }
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x3200, 0x03,
                                  JSD_SDO_DATA_FLOAT,
                                  (void*)&measuring_rate_hz)) {
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