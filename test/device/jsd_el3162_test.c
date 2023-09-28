#include <assert.h>
#include <string.h>

#include "jsd/jsd_el3162_pub.h"
#include "jsd/jsd_el3162_types.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;

void telemetry_header() {
  if (!file) {
    return;
  }
  for (int i = 0; i < JSD_EL3162_NUM_CHANNELS; ++i) {
    fprintf(file, "EL3162_ch%d_raw_value, EL3162_ch_%d_volts, ", i, i);
    fprintf(file, "error_ch%d, underrange_ch%d, overrange_ch%d, ", i, i, i);
  }
  fprintf(file, "\n");
}

void telemetry_data(void* self) {
  assert(self);

  if (!file) {
    return;
  }

  single_device_server_t*   sds   = (single_device_server_t*)self;
  const jsd_el3162_state_t* state = jsd_el3162_get_state(sds->jsd, slave_id);

  for (int i = 0; i < JSD_EL3162_NUM_CHANNELS; ++i) {
    fprintf(file, "%i, %lf,", state->adc_value[i], state->voltage[i]);
    fprintf(file, "%u, %u,", state->error[i], state->underrange[i]);
    fprintf(file, "%u,", state->overrange[i]);
  }
  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);

  single_device_server_t*   sds   = (single_device_server_t*)self;
  const jsd_el3162_state_t* state = jsd_el3162_get_state(sds->jsd, slave_id);
  MSG("Ch0: %f V, Ch1: %f V", state->voltage[0], state->voltage[1]);
}

void extract_data(void* self) {
  assert(self);

  single_device_server_t* sds = (single_device_server_t*)self;
  jsd_el3162_read(sds->jsd, slave_id);
}

void command(void* self) { (void)self; };

int main(int argc, char* argv[]) {
  if (argc != 4) {
    ERROR("Expecting exactly 3 arguments");
    MSG("Usage: jsd_el3162_test <ifname> <el3162_slave_index> <loop_freq_hz>");
    MSG("Example: $ jsd_el3162_test eth0 2 1000");
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

  // Set device configuration here.
  jsd_slave_config_t my_config = {0};

  snprintf(my_config.name, JSD_NAME_LEN, "unicorn");
  my_config.configuration_active = true;
  my_config.driver_type          = JSD_DRIVER_TYPE_EL3162;

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  sds_run(&sds, ifname, "/tmp/jsd_el3162.csv");

  free(ifname);

  return 0;
}