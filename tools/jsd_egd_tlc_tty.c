#include <float.h>
#include <inttypes.h>
#include <pthread.h>
#include <readline/history.h>
#include <readline/readline.h>
#include <signal.h>
#include <stdio.h>  // this include needs to come before readline/readline.h

#include "jsd/jsd.h"
#include "jsd/jsd_egd_pub.h"
#include "jsd/jsd_sdo_pub.h"

// Global variables
pthread_mutex_t mut = PTHREAD_MUTEX_INITIALIZER;
uint16_t        slave_id;
bool            quit = false;

void on_signal(int signal) {
  (void)signal;
  MSG("Received ctrl-c. Closing down.");
  quit = true;
}

void* tlc_tty_cb(void* jsd_ptr) {
  jsd_t* jsd = (jsd_t*)jsd_ptr;

  char                line[64];
  uint16_t            subindex;
  char                tlc[8];
  char                val_str[64];
  char                val_type[64];
  jsd_sdo_data_type_t data_type;
  jsd_sdo_data_t      sdo_data;

  while (!quit) {
    char* temp_line = readline(">> ");
    snprintf(line, 64, "%s", temp_line);
    free(temp_line);
    MSG_DEBUG("user input is: %s", line);

    if (0 == strcmp(line, "q") || 0 == strcmp(line, "quit")) {
      MSG("Quitting tty program");
      quit = true;
      continue;
    }

    int n_parsed = sscanf(line, "%s %[A-Z][%" SCNu16 "] = %s", val_type, tlc,
                          &subindex, val_str);

    if (n_parsed == 3) {  // SDO Read
      MSG_DEBUG("\t parsed val_type: %s", val_type);
      MSG_DEBUG("\t parsed tlc: %s", tlc);
      MSG_DEBUG("\t parsed subindex: %u", subindex);

      if (0 == strcmp(val_type, "float")) {
        data_type = JSD_SDO_DATA_FLOAT;

      } else if (0 == strcmp(val_type, "uint32_t")) {
        data_type = JSD_SDO_DATA_U32;

      } else if (0 == strcmp(val_type, "int32_t")) {
        data_type = JSD_SDO_DATA_I32;

      } else {
        WARNING("type %s not handled", val_type);
      }

      jsd_sdo_get_param_async(jsd, slave_id, jsd_egd_tlc_to_do(tlc), subindex,
                              data_type);

    } else if (n_parsed == 4) {  // SDO write
      MSG_DEBUG("\t parsed val_type: %s", val_type);
      MSG_DEBUG("\t parsed tlc: %s", tlc);
      MSG_DEBUG("\t parsed subindex: %u", subindex);
      MSG_DEBUG("\t parsed val_str: %s", val_str);

      if (0 == strcmp(val_type, "float")) {
        data_type         = JSD_SDO_DATA_FLOAT;
        sdo_data.as_float = atof(val_str);

      } else if (0 == strcmp(val_type, "int32_t")) {
        data_type       = JSD_SDO_DATA_U32;
        sdo_data.as_u32 = atoi(val_str);

      } else if (0 == strcmp(val_type, "uint32_t")) {
        data_type       = JSD_SDO_DATA_I32;
        sdo_data.as_i32 = atoi(val_str);

      } else {
        WARNING("Could not understand value type");
        WARNING("Should be {float, uint32_t, or int32_t}");
      }

      jsd_sdo_set_param_async(jsd, slave_id, jsd_egd_tlc_to_do(tlc), subindex,
                              data_type, &sdo_data);
    } else {
      WARNING("Could not parse user input, n_parsed: %d", n_parsed);
      MSG("Please input commands of the form: float CL[2] = 1.5");
      MSG("Or query current parameters using the form: float CL[2]");
      MSG("type q or quit (or ctrl+c) to end program");
      continue;
    }

    // only add if it was a valid command
    if (strlen(line) > 0) {
      add_history(line);
    }
  }
  return NULL;
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    ERROR("Expecting exactly 2 arguments");
    MSG("Usage: jsd_egd_tlc_tty <ifname> <egd_slave_index> ");
    MSG("Example: $ jsd_egd_tlc_tty eth0 2");
    return 0;
  }

  char* ifname = strdup(argv[1]);
  slave_id     = atoi(argv[2]);

  jsd_t* jsd;
  jsd = jsd_alloc();

  jsd_slave_config_t my_config = {0};

  snprintf(my_config.name, JSD_NAME_LEN, "my_egd");
  my_config.configuration_active         = true;
  my_config.product_code                 = JSD_EGD_PRODUCT_CODE;
  my_config.egd.drive_cmd_mode           = JSD_EGD_DRIVE_CMD_MODE_PROFILED;
  my_config.egd.max_motor_speed          = INT32_MAX;  // effectively disables
  my_config.egd.loop_period_ms           = 100;        // dummy
  my_config.egd.torque_slope             = 1e7;
  my_config.egd.max_profile_accel        = 1e6;
  my_config.egd.max_profile_decel        = 1e7;
  my_config.egd.velocity_tracking_error  = 1e8;  // Elmo Default
  my_config.egd.position_tracking_error  = 1e9;  // Elmo Default
  my_config.egd.peak_current_limit       = 1;    // dummy
  my_config.egd.peak_current_time        = 3.0;
  my_config.egd.continuous_current_limit = 1;  // dummy
  my_config.egd.motor_stuck_current_level_pct = 0;
  my_config.egd.motor_stuck_timeout           = 3.0;
  my_config.egd.over_speed_threshold          = 0;
  my_config.egd.low_position_limit            = 0;
  my_config.egd.high_position_limit           = 0;
  my_config.egd.crc                           = INT32_MIN;
  my_config.egd.drive_max_current_limit       = -FLT_MAX;

  jsd_set_slave_config(jsd, slave_id, my_config);

  // slave configuration must come before initialization
  if (!jsd_init(jsd, ifname, 1)) {
    ERROR("Could not init jsd");
    return 0;
  }

  // add signal handling
  signal(SIGINT, on_signal);

  // spool up tty thread
  pthread_t tlc_tty_thread;
  int       retval = pthread_create(&tlc_tty_thread, NULL, tlc_tty_cb, jsd);
  if (retval) {
    ERROR("Thread creation failed: %d\n", retval);
    return 1;
  }

  // process until interrupted
  while (!quit) {
    pthread_mutex_lock(&mut);
    jsd_read(jsd, EC_TIMEOUTRET);

    jsd_egd_read(jsd, slave_id);
    jsd_egd_process(jsd, slave_id);

    jsd_write(jsd);
    pthread_mutex_unlock(&mut);

    usleep(1e4);  // ~10 hz
  }

  jsd_free(jsd);
  SUCCESS("End jsd_egd_tty program");
  return 0;
}
