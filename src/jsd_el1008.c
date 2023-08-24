#include "jsd/jsd_el1008.h"

#include <assert.h>
#include <string.h>

#include "jsd/jsd_sdo.h"

/****************************************************
 * Public functions
 ****************************************************/

const jsd_el1008_state_t* jsd_el1008_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL1008_PRODUCT_CODE);

  jsd_el1008_state_t* state = &self->slave_states[slave_id].el1008;
  return state;
}

void jsd_el1008_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL1008_PRODUCT_CODE);

  jsd_el1008_state_t* state = &self->slave_states[slave_id].el1008;

  const jsd_el1008_txpdo_t* txpdo =
      (jsd_el1008_txpdo_t*)self->ecx_context.slavelist[slave_id].inputs;

  for (int ch = 0; ch < JSD_EL1008_NUM_CHANNELS; ++ch) {
    state->values[ch] = (bool)((1 << ch) & txpdo->channel[0].values); // Bit shift the channel to get value
  }
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el1008_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL1008_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  slave->PO2SOconfigx = jsd_el1008_PO2SO_config;

  return true;
}

int jsd_el1008_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_EL1008_PRODUCT_CODE);

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within the ecx_context and extract it here.
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;

  jsd_slave_config_t* config = &slave_configs[slave_id];

  MSG("Configuring slave no: %u, SII inferred name: %s", slave_id,
      ecx_context->slavelist[slave_id].name);
  MSG("\t Configured name: %s", config->name);

  config->PO2SO_success = true;
  return 1;
}
