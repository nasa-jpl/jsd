#include "jsd/jsd.h"

#include <assert.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "jsd/jsd_ati_fts.h"
#include "jsd/jsd_egd.h"
#include "jsd/jsd_el1008.h"
#include "jsd/jsd_el2124.h"
#include "jsd/jsd_el2809.h"
#include "jsd/jsd_el3104.h"
#include "jsd/jsd_el3162.h"
#include "jsd/jsd_el3202.h"
#include "jsd/jsd_el3208.h"
#include "jsd/jsd_el3318.h"
#include "jsd/jsd_el3356.h"
#include "jsd/jsd_el3602.h"
#include "jsd/jsd_el4102.h"
#include "jsd/jsd_epd_nominal.h"
#include "jsd/jsd_epd_sil.h"
#include "jsd/jsd_ild1900.h"
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

bool jsd_init(jsd_t* self, const char* ifname, uint8_t enable_autorecovery, int timeout_us) {
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
  int iomap_size =
      ecx_config_overlap_map_group(&self->ecx_context, &self->IOmap, 0);
  if (iomap_size > (int)sizeof(self->IOmap)) {
    ERROR("IO Map is not large enough for this application");
    return false;
  }
  // Print the IOMap input and output pointers for debugging
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
  ecx_readstate(&self->ecx_context);

  self->expected_wkc = (self->ecx_context.grouplist[0].outputsWKC * 2) +
                       self->ecx_context.grouplist[0].inputsWKC;

  self->last_wkc = -1;  // -1 is returned on first read

  MSG_DEBUG("Calculated workcounter %d", self->expected_wkc);

  MSG_DEBUG("Attempting to put the bus in Operational state");

  MSG_DEBUG("Performing first PDO exchange, required before transition to OP");

  self->ecx_context.slavelist[0].state = EC_STATE_OPERATIONAL;

  struct timespec start_processdata_time;
  clock_gettime(CLOCK_REALTIME, &start_processdata_time);
  ecx_send_overlap_processdata(&self->ecx_context);
  ecx_receive_processdata(&self->ecx_context, timeout_us);

  ecx_writestate(&self->ecx_context, 0);

  int attempt = 0;
  while (true) {
    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);
    if ((current_time.tv_nsec - start_processdata_time.tv_nsec)/1e3 > timeout_us) {
      MSG_DEBUG("Went over the loop period!");
    }
    else {
      struct timespec diff;
      diff.tv_sec = current_time.tv_sec - start_processdata_time.tv_sec;
      diff.tv_nsec = current_time.tv_nsec - start_processdata_time.tv_nsec;
      if (nanosleep(&diff, NULL) < 0) {
        perror("nanosleep failed");
        return 1;
      }
    }
    
    clock_gettime(CLOCK_REALTIME, &start_processdata_time);
    int sent = ecx_send_overlap_processdata(&self->ecx_context);
    int wkc  = ecx_receive_processdata(&self->ecx_context, timeout_us);
    ec_state actual_state = ecx_statecheck(
        &self->ecx_context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

    attempt++;

    MSG_DEBUG("sent: %d", sent);
    MSG_DEBUG("Actual WKC: %d, Expected WKC: %d", wkc, self->expected_wkc);

    if (actual_state != EC_STATE_OPERATIONAL) {
      WARNING("Did not reach %s, actual state is %s",
              jsd_ec_state_to_string(EC_STATE_OPERATIONAL),
              jsd_ec_state_to_string(actual_state));
      if (sent <= 0) {
        WARNING("Process data could not be transmitted properly.");
      }
      if (wkc != self->expected_wkc) {
        WARNING("Process data was not received properly.");
      }
      WARNING("Failed OP transition attempt %d of %d", attempt,
              JSD_PO2OP_MAX_ATTEMPTS);

      if (attempt >= JSD_PO2OP_MAX_ATTEMPTS) {
        ERROR("Max number of attempts to transition to OPERATIONAL exceeded.");
        return false;
      }
    } else {  // good to go
      break;
    }
  }

  // Initialize the error queues used between threads
  for (sid = 1; sid <= *self->ecx_context.slavecount; sid++) {
    char qname[JSD_NAME_LEN];
    snprintf(qname, JSD_NAME_LEN, "Slave %d error cirq", sid);
    jsd_error_cirq_init(&self->slave_errors[sid], qname);
  }

  // Initialize the sdo request/response queues and start background SDO thread
  jsd_sdo_req_cirq_init(&self->jsd_sdo_req_cirq, "Request Queue");
  jsd_sdo_req_cirq_init(&self->jsd_sdo_res_cirq, "Response Queue");

  // Make sure to only start this after the PO2OP hooks have completed
  if (0 != pthread_create(&self->sdo_thread, NULL, sdo_thread_loop, (void*)self)) {
    ERROR("Failed to create SDO thread");
    return false;
  }
  self->init_complete = true;

  SUCCESS("JSD is Operational");

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

  if(self->raise_sdo_thread_cond){
    pthread_cond_signal(&self->sdo_thread_cond);
    self->raise_sdo_thread_cond = false;
  }

}

void jsd_write(jsd_t* self) {
  assert(self);

  // Write EtherCat frame to slaves, with logic for smart prints
  int transmitted = ecx_send_overlap_processdata(&self->ecx_context);

  static int last_transmitted = 1;
  if (transmitted <= 0 && last_transmitted != transmitted) {
    WARNING("ecx_send_overlap_processdata is not transmitting");
  }
  if (last_transmitted <= 0 && transmitted > 0) {
    MSG("ecx_send_overlap_processdata has resumed transmission");
  }
  last_transmitted = transmitted;
}

void jsd_free(jsd_t* self) {
  if (!self) {
    return;
  }

  if(self->init_complete){
    struct timespec ts;

    self->sdo_join_flag = true;
    MSG("Waiting for SDO Thread to join...");
    pthread_cond_signal(&self->sdo_thread_cond);

    // The following loop should be more robust than just 
    //   a pthread_join blocking wait
    while(true) {
      self->sdo_join_flag = true;

      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_sec += 1;

      if(pthread_timedjoin_np(self->sdo_thread, NULL, &ts) == 0){
        break;
      }
    }
  }

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

/****************************************************
 * Private functions
 ****************************************************/

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

const char* jsd_driver_type_to_string(jsd_driver_type_t driver_type) {
  switch (driver_type) {
    case JSD_DRIVER_TYPE_ATI_FTS:
      return "JSD_DRIVER_TYPE_ATI_FTS";
    case JSD_DRIVER_TYPE_EGD:
      return "JSD_DRIVER_TYPE_EGD";
    case JSD_DRIVER_TYPE_EL1008:
      return "JSD_DRIVER_TYPE_EL1008";
    case JSD_DRIVER_TYPE_EL2124:
      return "JSD_DRIVER_TYPE_EL2124";
    case JSD_DRIVER_TYPE_EL2809:
      return "JSD_DRIVER_TYPE_EL2809";
    case JSD_DRIVER_TYPE_EL3104:
      return "JSD_DRIVER_TYPE_EL3104";
    case JSD_DRIVER_TYPE_EL3162:
      return "JSD_DRIVER_TYPE_EL3162";
    case JSD_DRIVER_TYPE_EL3202:
      return "JSD_DRIVER_TYPE_EL3202";
    case JSD_DRIVER_TYPE_EL3208:
      return "JSD_DRIVER_TYPE_EL3208";
    case JSD_DRIVER_TYPE_EL3318:
      return "JSD_DRIVER_TYPE_EL3318";
    case JSD_DRIVER_TYPE_EL3356:
      return "JSD_DRIVER_TYPE_EL3356";
    case JSD_DRIVER_TYPE_EL3602:
      return "JSD_DRIVER_TYPE_EL3602";
    case JSD_DRIVER_TYPE_EL4102:
      return "JSD_DRIVER_TYPE_EL4102";
    case JSD_DRIVER_TYPE_EPD_NOMINAL:
      return "JSD_DRIVER_TYPE_EPD_NOMINAL";
    case JSD_DRIVER_TYPE_EPD_SIL:
      return "JSD_DRIVER_TYPE_EPD_SIL";
    case JSD_DRIVER_TYPE_ILD1900:
      return "JSD_DRIVER_TYPE_ILD1900";
    case JSD_DRIVER_TYPE_JED0101:
      return "JSD_DRIVER_TYPE_JED0101";
    case JSD_DRIVER_TYPE_JED0200:
      return "JSD_DRIVER_TYPE_JED0200";
    default:
      return "Unknown Driver Type";
  }
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
    jsd_driver_type_t driver_type = self->slave_configs[slave_idx].driver_type;
    if (!jsd_driver_is_compatible_with_product_code(driver_type,
                                                    slave->eep_id)) {
      ERROR(
          "User-specified driver type (%s) is incompatible with the device's "
          "product code (%u).",
          jsd_driver_type_to_string(driver_type), slave->eep_id);
      ERROR("Not configuring this device, check your configuration!");
      return false;
    }

    if (!jsd_init_single_device(self, slave_idx)) {
      ERROR("\tBad Cfg  slave[%u] %s Active Device configuration mismatch!",
            slave_idx, slave->name);
      return false;
    }

    // EGDs nor EPDs have the name field populated.
    if (driver_type == JSD_DRIVER_TYPE_EGD) {
      SUCCESS("\tslave[%u] Elmo Gold Drive - Configured", slave_idx);
    } else if (driver_type == JSD_DRIVER_TYPE_EPD_NOMINAL) {
      SUCCESS("\tslave[%u] Elmo Platinum Drive in Nominal mode - Configured",
              slave_idx);
    } else if (driver_type == JSD_DRIVER_TYPE_EPD_SIL) {
      SUCCESS("\tslave[%u] Elmo Platinum Drive in SIL mode - Configured",
              slave_idx);
    } else {
      SUCCESS("\tslave[%u] %s - Configured", slave_idx, slave->name);
    }
  }

  return true;
}

bool jsd_driver_is_compatible_with_product_code(jsd_driver_type_t driver_type,
                                                uint32_t product_code) {
  bool is_compatible = false;
  switch (driver_type) {
    case JSD_DRIVER_TYPE_EL3602:
      is_compatible = jsd_el3602_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EL3208:
      is_compatible = jsd_el3208_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EL2124:
      is_compatible = jsd_el2124_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EL1008:
      is_compatible = jsd_el1008_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EL2809:
      is_compatible = jsd_el2809_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EGD:
      is_compatible = jsd_egd_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EL3356:
      is_compatible = jsd_el3356_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_JED0101:
      is_compatible = jsd_jed0101_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_JED0200:
      is_compatible = jsd_jed0200_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_ATI_FTS:
      is_compatible = jsd_ati_fts_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EL3104:
      is_compatible = jsd_el3104_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EL3202:
      is_compatible = jsd_el3202_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EL3318:
      is_compatible = jsd_el3318_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EL3162:
      is_compatible = jsd_el3162_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EL4102:
      is_compatible = jsd_el4102_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_ILD1900:
      is_compatible = jsd_ild1900_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EPD_NOMINAL:
      is_compatible = jsd_epd_nominal_product_code_is_compatible(product_code);
      break;
    case JSD_DRIVER_TYPE_EPD_SIL:
      is_compatible = jsd_epd_sil_product_code_is_compatible(product_code);
      break;
    default:
      ERROR("Invalid driver type (%i)", driver_type);
      assert(false);
  }
  return is_compatible;
}

bool jsd_init_single_device(jsd_t* self, uint16_t slave_id) {
  assert(self);

  jsd_driver_type_t driver_type = self->slave_configs[slave_id].driver_type;

  switch (driver_type) {
    case JSD_DRIVER_TYPE_EL3602:
      return jsd_el3602_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EL3208:
      return jsd_el3208_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EL3202:
      return jsd_el3202_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EGD:
      return jsd_egd_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EL1008:
      return jsd_el1008_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EL2809:
      return jsd_el2809_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EL2124:
      return jsd_el2124_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EL3356:
      return jsd_el3356_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_JED0101:
      return jsd_jed0101_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_JED0200:
      return jsd_jed0200_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_ATI_FTS:
      return jsd_ati_fts_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EL3104:
      return jsd_el3104_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EL3318:
      return jsd_el3318_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EL3162:
      return jsd_el3162_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EL4102:
      return jsd_el4102_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_ILD1900:
      return jsd_ild1900_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EPD_NOMINAL:
      return jsd_epd_nominal_init(self, slave_id);
      break;
    case JSD_DRIVER_TYPE_EPD_SIL:
      return jsd_epd_sil_init(self, slave_id);
      break;
    default:
      ERROR("Invalid driver type: %u", driver_type);
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
