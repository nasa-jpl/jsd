#include "jsd/jsd_jed0200.h"

#include <assert.h>

#include "jsd/jsd.h"
#include "jsd/jsd_sdo.h"

/****************************************************
 * Public functions
 ****************************************************/

const jsd_jed0200_state_t* jsd_jed0200_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_JED0200_PRODUCT_CODE);

  jsd_jed0200_state_t* state = &self->slave_states[slave_id].jed0200;
  return state;
}

void jsd_jed0200_set_cmd_value(jsd_t* self, uint16_t slave_id, uint16_t cmd) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_JED0200_PRODUCT_CODE);

  jsd_jed0200_state_t* state = &self->slave_states[slave_id].jed0200;
  state->cmd             = cmd;
}

void jsd_jed0200_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_JED0200_PRODUCT_CODE);

  jsd_jed0200_state_t* state = &self->slave_states[slave_id].jed0200;
  jsd_jed0200_txpdo_t* txpdo =
      (jsd_jed0200_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  state->status = txpdo->status;
  state->ticks = txpdo->ticks;
  state->voltage_hv = txpdo->voltage_hv;
  state->voltage_lv = txpdo->voltage_lv;
  state->voltage_12v = txpdo->voltage_12v;
  state->temp_ambient = txpdo->temp_ambient;
  state->temp_actuator = txpdo->temp_actuator;
  state->humidity = txpdo->humidity;
  state->pressure = txpdo->pressure;
  state->brake_current = txpdo->brake_current;
  state->brake_cc_val = txpdo->brake_cc_val;
}

void jsd_jed0200_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_JED0200_PRODUCT_CODE);

  jsd_jed0200_state_t* state = &self->slave_states[slave_id].jed0200;
  jsd_jed0200_rxpdo_t* rxpdo =
      (jsd_jed0200_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;

  rxpdo->cmd = state->cmd;

  jsd_async_sdo_process_response(self, slave_id);
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_jed0200_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_JED0200_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man == JSD_JPL_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  jsd_slave_config_t* config = &self->slave_configs[slave_id];

  slave->PO2SOconfigx   = jsd_jed0200_PO2SO_config;
  config->PO2SO_success = false;  // only set true in PO2SO callback

  jsd_jed0200_state_t* state = &self->slave_states[slave_id].jed0200;
  state->cmd             = config->jed0200.initial_cmd;

  return true;
}

int jsd_jed0200_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;
  jsd_slave_config_t* config = &slave_configs[slave_id];

  // There were initial issues with the Sync Managers having the wrong address
  uint8_t SM0, SM1, SM2, SM3;
  jsd_sdo_get_param_blocking(ecx_context, slave_id, 0x1C00, 1, JSD_SDO_DATA_U8,
                             &SM0);
  jsd_sdo_get_param_blocking(ecx_context, slave_id, 0x1C00, 2, JSD_SDO_DATA_U8,
                             &SM1);
  jsd_sdo_get_param_blocking(ecx_context, slave_id, 0x1C00, 3, JSD_SDO_DATA_U8,
                             &SM2);
  jsd_sdo_get_param_blocking(ecx_context, slave_id, 0x1C00, 4, JSD_SDO_DATA_U8,
                             &SM3);
  if (SM0 != 1 || SM1 != 2 || SM2 != 3 || SM3 != 4) {
    ERROR("Sync Manager are configured incorrectly, check the 0x1C00 register");
    config->PO2SO_success = false;
  } else {
    config->PO2SO_success = true;
    SUCCESS("JED0200[%d] drive parameters successfully configured and verified",
            slave_id);
  }
  return 1;
}
