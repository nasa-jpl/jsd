#include "jsd/jsd_el5042.h"

#include <assert.h>
#include <string.h>

#include "jsd/jsd_sdo.h"

const char* jsd_el5042_clock_strings[] = {
    [JSD_EL5042_10MHz]   = "10MHz Frequency",
    [JSD_EL5042_5MHz]    = "5MHz Frequency",
    [JSD_EL5042_3_33MHz]  = "3.33MHz Frequency",
    [JSD_EL5042_2_5MHz] = "2.5MHz Frequency",
    [JSD_EL5042_2MHz]  = "2MHz Frequency",
    [JSD_EL5042_1MHz] = "1MHz Frequency",
    [JSD_EL5042_500KHz] = "500kHz Frequency",
    [JSD_EL5042_250KHz] = "250kHz Frequency",
};

/****************************************************
 * Public functions
 ****************************************************/

const jsd_el5042_state_t* jsd_el5042_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el5042_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_el5042_state_t* state = &self->slave_states[slave_id].el5042;
  return state;
}

void jsd_el5042_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el5042_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));

  jsd_el5042_state_t* state = &self->slave_states[slave_id].el5042;

  const jsd_el5042_txpdo_t* txpdo =
      (jsd_el5042_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  for (int ch = 0; ch < JSD_EL5042_NUM_CHANNELS; ++ch) {
    state->position[ch] = txpdo->channel[ch].position;

    state->warning[ch]             = (txpdo->channel[ch].status >> 0) & 0x01;
    state->error[ch]               = (txpdo->channel[ch].status >> 1) & 0x01;
    state->ready[ch]               = (txpdo->channel[ch].status >> 2) & 0x01;
    state->diag[ch]                = (txpdo->channel[ch].status >> 4) & 0x01;
    state->txpdo_state[ch]         = (txpdo->channel[ch].status >> 5) & 0x01;
    state->input_cycle_counter[ch] = (txpdo->channel[ch].status >> 6) & 0x03;
  }
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el5042_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(jsd_el5042_product_code_is_compatible(
      self->ecx_context.slavelist[slave_id].eep_id));
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_el5042_PO2SO_config;

  return true;
}

int jsd_el5042_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(jsd_el5042_product_code_is_compatible(
      ecx_context->slavelist[slave_id].eep_id));

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within the ecx_context and extract it here.
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;

  jsd_slave_config_t* config = &slave_configs[slave_id];

  for (int ch = 0; ch < JSD_EL5042_NUM_CHANNELS; ++ch) {
    // Index for settings is 0x80n8, where n is channel number (e.g. ch2 =
    // 0x8018).
    uint32_t sdo_channel_index = 0x8008 + (0x10 * ch);

    // Negates the position value 
    uint8_t invert_feedback_direction = config->el5042.invert_feedback_direction[ch]; // 5V (default)
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x01, JSD_SDO_DATA_U8, &invert_feedback_direction)) {
      return 0;
    }

    // Tell the slave whether or not to send status bits 
    uint8_t disable_status_bits = config->el5042.disable_status_bits[ch]; 
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x02, JSD_SDO_DATA_U8, &disable_status_bits)) {
      return 0;
    }

    // Inverts the checksum bits (CRC) received by the encoder
    uint8_t invert_checksum = config->el5042.invert_checksum[ch]; 
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x03, JSD_SDO_DATA_U8, &invert_checksum)) {
      return 0;
    }

    // Polynomial used for calculating checksum
    uint32_t checksum_polynomial = config->el5042.checksum_polynomial[ch]; 
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x11, JSD_SDO_DATA_U8, &checksum_polynomial)) {
      return 0;
    }

    // Set the encoder supply voltage, either 50 for 5V or 90 for 9V
    uint8_t supply_voltage = config->el5042.supply_voltage[ch]; // 5V (default)
    if (supply_voltage != 50 && supply_voltage != 90) {
      MSG("Attempt to set supply voltage on channel %d to value of %d. "
          "Only a value of 50 (for 5V) and 90 (for 9V) is permitted!", ch, supply_voltage);
      return 0;
    }
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x12, JSD_SDO_DATA_U8, &supply_voltage)) {
      return 0;
    }

    // Clock frequency for the BiSS-C protocol
    uint8_t clock_frequency = config->el5042.clock_frequency[ch];
    MSG("\t clock[%d]: %s", ch, jsd_el5042_clock_strings[clock_frequency]); 
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x13, JSD_SDO_DATA_U8, &clock_frequency)) {
      return 0;
    }

    // Option to either use dual code (0) or gray code (1) for accurate data
    uint8_t gray_code = config->el5042.gray_code[ch]; 
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x14, JSD_SDO_DATA_U8, &gray_code)) {
      return 0;
    }

    // Set the number of multiturn bits (how many complete rotations)
    uint8_t multiturn_bits = config->el5042.multiturn_bits[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x15, JSD_SDO_DATA_U8, &multiturn_bits)) {
      return 0;
    }

    // Set the number of singleturn bits (resolution of a single rotation)
    uint8_t singleturn_bits = config->el5042.singleturn_bits[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x16, JSD_SDO_DATA_U8, &singleturn_bits)) {
      return 0;
    }

    // If there are addition null bits at the end of the packet, we shift by offset bits to get data
    uint8_t offset_bits = config->el5042.offset_bits[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x17, JSD_SDO_DATA_U8, &offset_bits)) {
      return 0;
    }

    // Opt for SSI mode (over BiSS-C)
    uint8_t ssi_mode = config->el5042.ssi_mode[ch];
    if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                    0x18, JSD_SDO_DATA_U8, &ssi_mode)) {
      return 0;
    }
  }

  config->PO2SO_success = true;
  return 1;
}

bool jsd_el5042_product_code_is_compatible(uint32_t product_code) {
  return product_code == JSD_EL5042_PRODUCT_CODE;
}
