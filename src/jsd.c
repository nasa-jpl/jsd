#include "jsd/jsd.h"

#include <assert.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "jsd/jsd_ati_fts.h"
#include "jsd/jsd_egd.h"
#include "jsd/jsd_el2124.h"
#include "jsd/jsd_el3104.h"
#include "jsd/jsd_el3202.h"
#include "jsd/jsd_el3208.h"
#include "jsd/jsd_el3318.h"
#include "jsd/jsd_el3356.h"
#include "jsd/jsd_el3602.h"
#include "jsd/jsd_jed0101.h"
#include "jsd/jsd_jed0200.h"
#include "jsd/jsd_print.h"
#include "jsd/jsd_sdo.h"

/****************************************************
 * Public functions
 ****************************************************/

jsd_t* jsd_alloc() {
  jsd_t* self;
  self = (jsd_t*)calloc(1, sizeof(jsd_t));

  self->ecx_context.port = (ecx_portt*)calloc(1, sizeof(ecx_portt));
  self->ecx_context.slavelist =
      (ec_slavet*)malloc(EC_MAXSLAVE * sizeof(ec_slavet));
  self->ecx_context.slavecount = (int*)calloc(1, sizeof(int));
  self->ecx_context.maxslave   = EC_MAXSLAVE;
  self->ecx_context.grouplist =
      (ec_groupt*)calloc(1, EC_MAXGROUP * sizeof(ec_groupt));
  self->ecx_context.maxgroup = EC_MAXGROUP;
  self->ecx_context.esibuf   = (uint8*)calloc(1, EC_MAXEEPBUF * sizeof(uint8));
  self->ecx_context.esimap =
      (uint32*)calloc(1, EC_MAXEEPBITMAP * sizeof(uint32));
  self->ecx_context.esislave  = 0;
  self->ecx_context.elist     = (ec_eringt*)calloc(1, sizeof(ec_eringt));
  self->ecx_context.idxstack  = (ec_idxstackT*)calloc(1, sizeof(ec_idxstackT));
  self->ecx_context.ecaterror = (boolean*)calloc(1, sizeof(boolean));
  self->ecx_context.DCtime    = (int64*)calloc(1, sizeof(int64));
  self->ecx_context.SMcommtype =
      (ec_SMcommtypet*)calloc(1, EC_MAX_MAPT * sizeof(ec_SMcommtypet));
  self->ecx_context.PDOassign =
      (ec_PDOassignt*)calloc(1, EC_MAX_MAPT * sizeof(ec_PDOassignt));
  self->ecx_context.PDOdesc =
      (ec_PDOdesct*)calloc(1, EC_MAX_MAPT * sizeof(ec_PDOdesct));
  self->ecx_context.eepSM = (ec_eepromSMt*)calloc(1, sizeof(ec_eepromSMt));
  self->ecx_context.eepFMMU =
      (ec_eepromFMMUt*)calloc(1, sizeof(ec_eepromFMMUt));
  self->ecx_context.FOEhook           = NULL;
  self->ecx_context.EOEhook           = NULL;
  self->ecx_context.manualstatechange = 0;
  // Needed to avoid global variables to pass slave config into the PO2SO
  // callbacks
  self->ecx_context.userdata = (void*)&self->slave_configs;

  return self;
}

void jsd_set_slave_config(jsd_t* self, uint16_t slave_id,
                          jsd_slave_config_t slave_config) {
  assert(self);
  assert(slave_id < EC_MAXSLAVE);

  if (self->init_complete) {
    ERROR("slave configuration must be set before jsd_init() is called");
  }
  assert(!self->init_complete);

  self->slave_configs[slave_id] = slave_config;
}

// TODO check more of these return status to ensure clean initialization
bool jsd_init(jsd_t* self, const char* ifname, uint8_t enable_autorecovery) {
  assert(self);
  self->enable_autorecovery = enable_autorecovery;

  if (ecx_init(&self->ecx_context, ifname) <= 0) {
    ERROR("Unable to establish socket connection on %s", ifname);
    if(geteuid() == 0) {
      ERROR("Is the device on and connected?");
    } else {
      ERROR("Execute as root");
    }
    return false;
  }

  MSG("ecx_init on %s succeeded", ifname);

  // find and auto-config slaves
  if (ecx_config_init(&self->ecx_context, FALSE) <= 0) {
    WARNING("No slaves found on %s", ifname);
    return false;
  }
  MSG("%d slaves found on bus", *self->ecx_context.slavecount + 1);

  // We need to register the SO2PO callbacks before mapping the PDOs
  if (!jsd_init_all_devices(self)) {
    ERROR("Could not init all devices");
    return false;
  }

  // configure IOMap
  int iomap_size = ecx_config_map_group(&self->ecx_context, &self->IOmap, 0);
  if (iomap_size > (int)sizeof(self->IOmap)) {
    ERROR("IO Map is not large enough for this application");
    return false;
  }
  // Print the IOMap input and output pointers for debugging
  // TODO remove when mature
  int sid;
  for (sid = 1; sid <= *self->ecx_context.slavecount; sid++) {
    MSG_DEBUG(
        "slave[%d] \tInputPtr: %p \tIBytes: %u \t IBits: %u \tOutputPtr: "
        "%p \tOBytes: %u \t OBits: %u",
        sid, self->ecx_context.slavelist[sid].inputs,
        self->ecx_context.slavelist[sid].Ibytes,
        self->ecx_context.slavelist[sid].Ibits,
        self->ecx_context.slavelist[sid].outputs,
        self->ecx_context.slavelist[sid].Obytes,
        self->ecx_context.slavelist[sid].Obits);
  }

  // Triggering the PO2SO transition that configures each device
  ecx_statecheck(&self->ecx_context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

  // verify the PO2SO callback executed completely
  for (sid = 1; sid <= *self->ecx_context.slavecount; sid++) {
    jsd_slave_config_t* config = &self->slave_configs[sid];
    if (config->configuration_active && !config->PO2SO_success) {
      ERROR("PO2SO callback did not execute successfully, failed on slave %d",
            sid);
      return false;
    }
  }
  // Auto-configure Distributed Clock capable slaves
  ecx_configdc(&self->ecx_context);

  // Read individual slave state and store in self->ecx_context.slavelist[]
  // TODO Why?
  ecx_readstate(&self->ecx_context);

  self->expected_wkc = (self->ecx_context.grouplist[0].outputsWKC * 2) +
                       self->ecx_context.grouplist[0].inputsWKC;

  self->last_wkc = -1;  // -1 is returned on first read

  MSG_DEBUG("Calculated workcounter %d", self->expected_wkc);

  MSG_DEBUG("Attempting to put the bus in Operational state");

  // if (!jsd_set_device_state(self, 0, EC_STATE_INIT, EC_TIMEOUTSTATE)) {
  //  ERROR("Could not reach INIT");
  //  return false;
  //}
  // if (!jsd_set_device_state(self, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE)) {
  //  ERROR("Could not reach PRE_OP");
  //  return false;
  //}
  // if (!jsd_set_device_state(self, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE)) {
  //  ERROR("Could not reach SAFE_OP");
  //  return false;
  //}

  MSG_DEBUG("Performing first PDO exchange, required before transition to OP");

  int attempt = 0;
  while (true) {
    int sent = ecx_send_processdata(&self->ecx_context);
    int wkc  = ecx_receive_processdata(&self->ecx_context, EC_TIMEOUTRET);

    attempt++;

    if (!jsd_set_device_state(self, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE)) {
      if (sent <= 0) {
        WARNING("Processdata could not be transmitted properly");
      }
      if (wkc != self->expected_wkc) {
        WARNING("Processdata was not received properly");
      }
      WARNING("Failed OP transition attempt %d of %d", attempt,
              JSD_PO2OP_MAX_ATTEMPTS);

      if (attempt > JSD_PO2OP_MAX_ATTEMPTS) {
        ERROR("Max number of attempts to transition to OPERATIONAL exceeded");
        return false;
      }
    } else {  // good to go
      break;
    }
  }

  if (0 !=
      pthread_create(&self->sdo_thread, NULL, sdo_thread_loop, (void*)self)) {
    ERROR("Failed to create SDO thread");
    return false;
  }

  jsd_sdo_req_cirq_init(&self->jsd_sdo_req_cirq);
  snprintf(self->jsd_sdo_req_cirq.name, JSD_NAME_LEN, "Request Queue");

  // technically, overallocates by 1 slave since sid=0 should not be used
  // leaving so slave_id can directly index this without offset
  int i;
  for (i = 0; i < *self->ecx_context.slavecount + 1; i++) {
    jsd_sdo_req_cirq_init(&self->jsd_sdo_res_cirq[i]);
    snprintf(self->jsd_sdo_res_cirq[i].name, JSD_NAME_LEN, "Response Queue %d",
             i);
  }

  SUCCESS("JSD is Operational");

  self->init_complete = true;
  return true;
}

// request state for all slaves is achieved by setting state on index 0
bool jsd_set_device_state(jsd_t* self, uint16_t slave_id, ec_state state,
                          int timeout_us) {
  assert(self);

  self->ecx_context.slavelist[slave_id].state = state;
  ecx_writestate(&self->ecx_context, slave_id);

  // wait for all slaves to reach desired state
  ec_state act_state =
      ecx_statecheck(&self->ecx_context, slave_id, state, timeout_us);
  ecx_statecheck(&self->ecx_context, slave_id, state, timeout_us);

  if (act_state != state) {
    WARNING("State Check failed. Requested = %s, Actual = %s",
            jsd_ec_state_to_string(state), jsd_ec_state_to_string(act_state));
    return false;
  }

  return true;
}

void jsd_read(jsd_t* self, int timeout_us) {
  assert(self);

  // Wait for EtherCat frame to return from slaves, with logic for smart prints
  self->wkc = ecx_receive_processdata(&self->ecx_context, timeout_us);
  if (self->wkc != self->expected_wkc && self->last_wkc != self->wkc) {
    WARNING("ecx_receive_processdata returning bad wkc: %d (expected: %d)",
            self->wkc, self->expected_wkc);
  }
  if (self->last_wkc != self->expected_wkc && self->wkc == self->expected_wkc) {
    if (self->last_wkc != -1) {
      MSG("ecx_receive_processdata is not longer reading bad wkc");
    }
  }
  self->last_wkc = self->wkc;

  if (self->enable_autorecovery || self->attempt_manual_recovery) {
    jsd_ecatcheck(self);
    self->attempt_manual_recovery = 0;
  }

  // Need to perform a read to get EMCY error updates
  ec_mbxbuft MbxIn;
  ecx_mbxreceive(&self->ecx_context, 0, (ec_mbxbuft*)&MbxIn, 0);
  // If there is an EMCY error, handle it
  if (ecx_iserror(&self->ecx_context)) {
    ec_errort error;
    ecx_poperror(&self->ecx_context, &error);

    char fault_string[JSD_NAME_LEN];
    switch (self->ecx_context.slavelist[error.Slave].eep_id) {
      case JSD_EGD_PRODUCT_CODE: {
        self->slave_states[error.Slave].egd.pub.fault_code =
            jsd_egd_get_fault_code_from_ec_error(error);
        snprintf(fault_string, JSD_NAME_LEN, "%s (0x%x)",
                 jsd_egd_fault_code_to_string(
                     self->slave_states[error.Slave].egd.pub.fault_code),
                 error.ErrorCode);
        break;
      }
      default:
        snprintf(fault_string, JSD_NAME_LEN, "Unknown Fault (0x%x)",
                 error.ErrorCode);
        break;
    }
    ERROR("EMCY: on slave id: %d, Description:  %s", error.Slave, fault_string);
  }
}

void jsd_write(jsd_t* self) {
  assert(self);

  // Write EtherCat frame to slaves, with logic for smart prints
  int transmitted = ecx_send_processdata(&self->ecx_context);

  static int last_transmitted = 1;
  if (transmitted <= 0 && last_transmitted != transmitted) {
    WARNING("ecx_send_processdata is not transmitting");
  }
  if (last_transmitted <= 0 && transmitted > 0) {
    MSG("ecx_send_processdata has resumed transmission");
  }
  last_transmitted = transmitted;
}

void jsd_free(jsd_t* self) {
  if (!self) {
    return;
  }

  self->sdo_join_flag = true;
  pthread_cond_signal(&self->sdo_thread_cond);
  MSG("Waiting for SDO Thread to join...");
  pthread_join(self->sdo_thread, NULL);

  MSG_DEBUG("Closing SOEM socket connection...");
  ecx_close(&self->ecx_context);

  free(self->ecx_context.port);
  free(self->ecx_context.slavelist);
  free(self->ecx_context.slavecount);
  free(self->ecx_context.grouplist);
  free(self->ecx_context.esibuf);
  free(self->ecx_context.esimap);
  free(self->ecx_context.elist);
  free(self->ecx_context.idxstack);
  free(self->ecx_context.ecaterror);
  free(self->ecx_context.DCtime);
  free(self->ecx_context.SMcommtype);
  free(self->ecx_context.PDOassign);
  free(self->ecx_context.PDOdesc);
  free(self->ecx_context.eepSM);
  free(self->ecx_context.eepFMMU);
  free(self);
  MSG_DEBUG("Freed JSD context");
}

ec_state jsd_get_device_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  return self->ecx_context.slavelist[slave_id].state;
}

void jsd_set_manual_recovery(jsd_t* self) {
  assert(self);
  self->attempt_manual_recovery = 1;
}

char* jsd_ec_state_to_string(ec_state state) {
  switch (state) {
    case EC_STATE_NONE:
      return "EC_STATE_NONE";
      break;
    case EC_STATE_PRE_OP:
      return "EC_STATE_PRE_OP";
      break;
    case EC_STATE_BOOT:
      return "EC_STATE_BOOT";
      break;
    case EC_STATE_SAFE_OP:
      return "EC_STATE_SAFE_OP";
      break;
    case EC_STATE_OPERATIONAL:
      return "EC_STATE_OPERATIONAL";
      break;
    case EC_STATE_ACK:
      return "EC_STATE_ACK/ERROR";
      break;
    default: {
      char str[JSD_NAME_LEN];
      snprintf(str, JSD_NAME_LEN, "Bad EC_STATE: 0x%x", state);
      return strdup(str);
      break;
    }
  }
}

/****************************************************
 * Private functions
 ****************************************************/

double jsd_get_time_sec() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

bool jsd_init_all_devices(jsd_t* self) {
  assert(self);

  uint16_t num_found_slaves = *(self->ecx_context.slavecount);
  assert(num_found_slaves < EC_MAXSLAVE);

  MSG("Configuring Slaves:");

  ec_slavet* slaves = self->ecx_context.slavelist;
  uint16_t   slave_idx;

  // slavecount does not include the 0 index virtual device master
  for (slave_idx = 1; slave_idx < num_found_slaves + 1; ++slave_idx) {
    ec_slavet* slave = &slaves[slave_idx];

    // Check user-provided configuration active flag
    if (!self->slave_configs[slave_idx].configuration_active) {
      MSG("\tslave[%u] %s - Ignored", slave_idx, slave->name);
      continue;
    }

    // Check the user-provided product code
    if (self->slave_configs[slave_idx].product_code != slave->eep_id) {
      ERROR("User product code (%u) does not match device product code (%u)",
            self->slave_configs[slave_idx].product_code, slave->eep_id);
      ERROR("Not configuring this device, check your configuration!");
      return false;
    }

    if (!jsd_init_single_device(self, slave_idx)) {
      ERROR("\tBad Cfg  slave[%u] %s Active Device configuration mismatch!",
            slave_idx, slave->name);
      return false;
    }

    // EGDs don't have the name field populated
    if (slave->eep_id == JSD_EGD_PRODUCT_CODE) {
      SUCCESS("\tslave[%u] Elmo Gold Drive - Configured", slave_idx);
    } else {
      SUCCESS("\tslave[%u] %s - Configured", slave_idx, slave->name);
    }
  }

  return true;
}

bool jsd_init_single_device(jsd_t* self, uint16_t slave_id) {
  assert(self);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];

  switch (slave->eep_id) {
    case JSD_EL3602_PRODUCT_CODE: {
      return jsd_el3602_init(self, slave_id);
      break;
    }
    case JSD_EL3208_PRODUCT_CODE: {
      return jsd_el3208_init(self, slave_id);
      break;
    }
    case JSD_EL3202_PRODUCT_CODE: {
      return jsd_el3202_init(self, slave_id);
      break;
    }
    case JSD_EGD_PRODUCT_CODE: {
      return jsd_egd_init(self, slave_id);
      break;
    }
    case JSD_EL2124_PRODUCT_CODE: {
      return jsd_el2124_init(self, slave_id);
      break;
    }
    case JSD_EL3356_PRODUCT_CODE: {
      return jsd_el3356_init(self, slave_id);
      break;
    }
    case JSD_JED0101_PRODUCT_CODE: {
      return jsd_jed0101_init(self, slave_id);
      break;
    }
    case JSD_JED0200_PRODUCT_CODE: {
      return jsd_jed0200_init(self, slave_id);
      break;
    }
    case JSD_ATI_FTS_PRODUCT_CODE: {
      return jsd_ati_fts_init(self, slave_id);
      break;
    }
    case JSD_EL3104_PRODUCT_CODE: {
      return jsd_el3104_init(self, slave_id);
      break;
    }
    case JSD_EL3318_PRODUCT_CODE: {
      return jsd_el3318_init(self, slave_id);
      break;
    }
    default:
      ERROR("Bad Product Code: %u", slave->eep_id);
      return false;
  }

  return true;
}

void jsd_ecatcheck(jsd_t* self) {
  uint8_t currentgroup = 0;  // only 1 rate group in JSD currently
  int     slave;

  ec_state bus_state = jsd_get_device_state(self, 0);

  if ((bus_state == EC_STATE_OPERATIONAL && self->wkc < self->expected_wkc) ||
      self->ecx_context.grouplist[currentgroup].docheckstate) {
    /* one ore more slaves are not responding */
    self->ecx_context.grouplist[currentgroup].docheckstate = FALSE;
    ecx_readstate(&self->ecx_context);
    for (slave = 1; slave <= *self->ecx_context.slavecount; slave++) {
      if ((self->ecx_context.slavelist[slave].group == currentgroup) &&
          (self->ecx_context.slavelist[slave].state != EC_STATE_OPERATIONAL)) {
        self->ecx_context.grouplist[currentgroup].docheckstate = TRUE;
        if (self->ecx_context.slavelist[slave].state ==
            (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
          MSG_DEBUG("slave[%d] is in SAFE_OP + ERROR, attempting ack.", slave);
          self->ecx_context.slavelist[slave].state =
              (EC_STATE_SAFE_OP + EC_STATE_ACK);
          ecx_writestate(&self->ecx_context, slave);
        } else if (self->ecx_context.slavelist[slave].state ==
                   EC_STATE_SAFE_OP) {
          MSG_DEBUG("slave[%d] is in SAFE_OP, changing to OPERATIONAL.", slave);
          self->ecx_context.slavelist[slave].state = EC_STATE_OPERATIONAL;
          ecx_writestate(&self->ecx_context, slave);
        } else if (self->ecx_context.slavelist[slave].state > EC_STATE_NONE) {
          if (ecx_reconfig_slave(&self->ecx_context, slave, EC_TIMEOUTRET3)) {
            self->ecx_context.slavelist[slave].islost = FALSE;
            MSG("slave[%d] was reconfigured", slave);
          }
        } else if (!self->ecx_context.slavelist[slave].islost) {
          /* re-check state */
          ecx_statecheck(&self->ecx_context, 0, EC_STATE_OPERATIONAL,
                         EC_TIMEOUTRET);
          if (self->ecx_context.slavelist[slave].state == EC_STATE_NONE) {
            self->ecx_context.slavelist[slave].islost = TRUE;
            ERROR("slave[%d] is lost", slave);
          }
        }
      }
      if (self->ecx_context.slavelist[slave].islost) {
        if (self->ecx_context.slavelist[slave].state == EC_STATE_NONE) {
          if (ecx_recover_slave(&self->ecx_context, slave, EC_TIMEOUTRET3)) {
            self->ecx_context.slavelist[slave].islost = FALSE;
            MSG("slave[%d] recovered", slave);
          }
        } else {
          self->ecx_context.slavelist[slave].islost = FALSE;
          MSG("slave %d found", slave);
        }
      }
    }
    if (!self->ecx_context.grouplist[currentgroup].docheckstate)
      SUCCESS("all slaves resumed OPERATIONAL.");
  }
}
