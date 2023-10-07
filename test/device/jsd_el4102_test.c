#include <assert.h>
#include <string.h>

#include "jsd/jsd_el4102_pub.h"
#include "jsd/jsd_el4102_types.h"
#include "jsd_test_utils.h"

extern FILE* file;
uint8_t      slave_id;
double       cmd_output[JSD_EL4102_NUM_CHANNELS] = {10.0, 5.0};

void telemetry_header() {
  if (!file) {
    return;
  }
  for (int i = 0; i < JSD_EL4102_NUM_CHANNELS; ++i) {
    fprintf(file, "EL4102_ch%d_raw_value, EL4102_ch%d_volts, ", i, i);
  }
  fprintf(file, "\n");
}

void telemetry_data(void* self) {
  assert(self);
  if (!file) {
    return;
  }

  single_device_server_t*   sds   = (single_device_server_t*)self;
  const jsd_el4102_state_t* state = jsd_el4102_get_state(sds->jsd, slave_id);

  for (int i = 0; i < JSD_EL4102_NUM_CHANNELS; ++i) {
    fprintf(file, "%i, %lf, ", state->dac_output[i], state->voltage_output[i]);
  }
  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  const jsd_el4102_state_t* state = jsd_el4102_get_state(sds->jsd, slave_id);

  MSG("Ch0: %f V, Ch1: %f V", state->voltage_output[0],
      state->voltage_output[1]);
}

void extract_data(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  jsd_el4102_process(sds->jsd, slave_id);
}

void command(void* self) {
  static uint32_t iter = 0;
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  // Swap values of channels every 5 seconds.
  if (iter % (sds->loop_rate_hz * 5) == 0) {
    double temp   = cmd_output[0];
    cmd_output[0] = cmd_output[1];
    cmd_output[1] = temp;
  }

  jsd_el4102_write_all_channels(sds->jsd, slave_id, cmd_output);
  ++iter;
}

int main(int argc, char const* argv[]) {
  if (argc != 4) {
    ERROR("Expecting exactly 3 arguments");
    MSG("Usage: jsd_el4102_test <ifname> <el4102_slave_index> <loop_freq_hz>");
    MSG("Example: $ jsd_el4102 eth0 5 10");
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

  // Set device configuration here.
  jsd_slave_config_t my_config = {0};

  snprintf(my_config.name, JSD_NAME_LEN, "nahual");
  my_config.configuration_active = true;
  my_config.driver_type          = JSD_DRIVER_TYPE_EL4102;

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  sds_run(&sds, ifname, "/tmp/jsd_el4102.csv");

  free(ifname);

  return 0;
}