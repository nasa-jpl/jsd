#include <assert.h>
#include <string.h>

#include "jsd/jsd_jed0200_pub.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;
uint16_t     cmd = 0;

void telemetry_header() {
  if (!file) {
    return;
  }

  fprintf(file, "status, ");
  fprintf(file, "w_raw, x_raw, y_raw, z_raw, ");
  fprintf(file, "w, x, y, z, ");
  fprintf(file, "cmd, ");
  fprintf(file, "\n");
}

void telemetry_data(void* self) {
  assert(self);
  if (!file) {
    return;
  }

  single_device_server_t* sds   = (single_device_server_t*)self;
  const jsd_jed0200_state_t*  state = jsd_jed0200_get_state(sds->jsd, slave_id);

  fprintf(file, "%u, ", state->status);
  fprintf(file, "%u, ", state->ticks);
  fprintf(file, "%u, ", state->voltage_hv);
  fprintf(file, "%u, ", state->voltage_lv);
  fprintf(file, "%u, ", state->voltage_12v);
  fprintf(file, "%u, ", state->temp_ambient);
  fprintf(file, "%u, ", state->temp_actuator);
  fprintf(file, "%u, ", state->humidity);
  fprintf(file, "%u, ", state->pressure);
  fprintf(file, "%u, ", state->brake_current);
  fprintf(file, "%u, ", state->brake_cc_val);
  fprintf(file, "%u, ", state->cmd);

  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);

  single_device_server_t* sds   = (single_device_server_t*)self;
  const jsd_jed0200_state_t*  state = jsd_jed0200_get_state(sds->jsd, slave_id);

  MSG("Status: %u (Qw: %lf, Qx: %lf, Qy: %lf, Qz: %lf) Cmd: %u", state->status,
      state->w, state->x, state->y, state->z, state->cmd);
}

void extract_data(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  jsd_jed0200_read(sds->jsd, slave_id);
  jsd_jed0200_process(sds->jsd, slave_id);
}

void command(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  jsd_jed0200_set_cmd_value(sds->jsd, slave_id, cmd++ % 255);
}

int main(int argc, char const* argv[]) {
  if (argc != 4) {
    ERROR("Expecting exactly 3 arguments");
    MSG("Usage: jsd_jed0200_test <ifname> <jed0200_slave_index> <loop_freq_hz>");
    MSG("Example: $ jsd_jed0200_test eth0 5 500");
    return 0;
  }

  char* ifname          = strdup(argv[1]);
  slave_id              = atoi(argv[2]);
  uint16_t loop_freq_hz = atoi(argv[3]);
  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using frequency of %u hz", loop_freq_hz);

  single_device_server_t sds;

  sds_set_telemetry_header_callback(&sds, telemetry_header);
  sds_set_telemetry_data_callback(&sds, telemetry_data);
  sds_set_print_info_callback(&sds, print_info);
  sds_set_extract_data_callback(&sds, extract_data);
  sds_set_command_callback(&sds, command);

  sds_setup(&sds, loop_freq_hz);

  // set device configuration here
  jsd_slave_config_t my_config = {0};

  snprintf(my_config.name, JSD_NAME_LEN, "bigfoot");
  my_config.configuration_active = true;
  my_config.product_code         = JSD_JED0200_PRODUCT_CODE;
  my_config.jed0200.initial_cmd      = 0;
  jsd_set_slave_config(sds.jsd, slave_id, my_config);
  sds_run(&sds, ifname, "/tmp/jsd_jed0200.csv");

  return 0;
}
