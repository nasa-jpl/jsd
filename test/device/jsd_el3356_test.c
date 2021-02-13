#include <assert.h>
#include <string.h>

#include "jsd/jsd_el3356_pub.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;

void telemetry_header() {
  if (!file) {
    return;
  }

  fprintf(file, "overrange, data_invalid, error, cal_in_prog, ");
  fprintf(file,
          "steady_state, sync_error, txpdo_toggle, value, pending_tare, ");
  fprintf(file, "scaled_value, ");
  fprintf(file, "\n");
}

void telemetry_data(void* self) {
  assert(self);

  if (!file) {
    return;
  }

  single_device_server_t*   sds   = (single_device_server_t*)self;
  const jsd_el3356_state_t* state = jsd_el3356_get_state(sds->jsd, slave_id);

  fprintf(file, "%u, ", state->overrange);
  fprintf(file, "%u, ", state->data_invalid);
  fprintf(file, "%u, ", state->error);
  fprintf(file, "%u, ", state->cal_in_prog);
  fprintf(file, "%u, ", state->steady_state);
  fprintf(file, "%u, ", state->sync_error);
  fprintf(file, "%u, ", state->txpdo_toggle);
  fprintf(file, "%d, ", state->value);
  fprintf(file, "%lf, ", state->scaled_value);

  fprintf(file, "%u, ", state->pending_tare);

  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);

  single_device_server_t*   sds   = (single_device_server_t*)self;
  const jsd_el3356_state_t* state = jsd_el3356_get_state(sds->jsd, slave_id);
  MSG("Value: %d Scaled_value: %lf", state->value, state->scaled_value);

  static int cnt = 0;
  cnt++;
  if (0 == cnt % 10) {
    MSG("Sending Tare to EL3356 module");
    jsd_el3356_tare(sds->jsd, slave_id);
  }
}

void extract_data(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  jsd_el3356_read(sds->jsd, slave_id);
  jsd_el3356_process(sds->jsd, slave_id);
}

void command(void* self) { (void)self; };

int main(int argc, char* argv[]) {
  if (argc != 4) {
    ERROR("Expecting exactly 3 arguments");
    MSG("Usage: jsd_el3356_test <ifname> <el3356_slave_index> <loop_freq_hz>");
    MSG("Example: $ jsd_el3356_test eth0 2 1000");
    return 0;
  }

  char* ifname          = strdup(argv[1]);
  slave_id              = atoi(argv[2]);
  uint32_t loop_freq_hz = atoi(argv[3]);
  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using frequency of %i hz", loop_freq_hz);

  single_device_server_t sds;

  sds_set_telemetry_header_callback(&sds, telemetry_header);
  sds_set_telemetry_data_callback(&sds, telemetry_data);
  sds_set_print_info_callback(&sds, print_info);
  sds_set_extract_data_callback(&sds, extract_data);
  sds_set_command_callback(&sds, command);

  sds_setup(&sds, loop_freq_hz);

  // set device configuration here
  jsd_slave_config_t my_config = {0};

  snprintf(my_config.name, JSD_NAME_LEN, "unicorn");
  my_config.configuration_active = true;
  my_config.product_code         = JSD_EL3356_PRODUCT_CODE;
  my_config.el3356.scale_factor  = 6.52965e-1;

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  sds_run(&sds, ifname, "/tmp/jsd_el3356.csv");

  return 0;
}
