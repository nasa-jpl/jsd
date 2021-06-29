
#include "jsd/jsd_el3356.h"

#include <assert.h>

#include "jsd/jsd_sdo.h"

/****************************************************
 * Public functions
 ****************************************************/

const jsd_el3356_state_t* jsd_el3356_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3356_PRODUCT_CODE);

  jsd_el3356_state_t* state = &self->slave_states[slave_id].el3356;
  return state;
}

void jsd_el3356_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3356_PRODUCT_CODE);

  jsd_el3356_state_t*  state  = &self->slave_states[slave_id].el3356;
  jsd_el3356_config_t* config = &self->slave_configs[slave_id].el3356;

  jsd_el3356_txpdo_t* txpdo =
      (jsd_el3356_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  state->overrange    = (txpdo->status_fields >> 1) & 0x01;
  state->data_invalid = (txpdo->status_fields >> 3) & 0x01;
  state->error        = (txpdo->status_fields >> 6) & 0x01;
  state->cal_in_prog  = (txpdo->status_fields >> 7) & 0x01;
  state->steady_state = (txpdo->status_fields >> 8) & 0x01;
  state->sync_error   = (txpdo->status_fields >> 13) & 0x01;
  state->txpdo_toggle = (txpdo->status_fields >> 15) & 0x01;
  state->value        = txpdo->value;
  state->scaled_value = (double)state->value * config->scale_factor;
}

void jsd_el3356_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3356_PRODUCT_CODE);

  jsd_el3356_state_t* state = &self->slave_states[slave_id].el3356;

  jsd_el3356_rxpdo_t* rxpdo =
      (jsd_el3356_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;

  rxpdo->cmd_fields = 0;

  if (state->pending_tare) {
    rxpdo->cmd_fields |= 0x01 << 0;  // Start calibration
    rxpdo->cmd_fields |= 0x01 << 4;  // Tare
    state->pending_tare = 0;
    MSG("Sending Tare");
  }

  jsd_async_sdo_process_response(self, slave_id);
}

void jsd_el3356_tare(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3356_PRODUCT_CODE);

  jsd_el3356_state_t* state = &self->slave_states[slave_id].el3356;
  state->pending_tare       = 1;
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el3356_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL3356_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_el3356_PO2SO_config;

  return true;
}

int jsd_el3356_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_EL3356_PRODUCT_CODE);

  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;
  jsd_slave_config_t* config = &slave_configs[slave_id];

  config->PO2SO_success = true;
  return 1;
}
