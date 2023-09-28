#include <assert.h>
#include <string.h>

#include "jsd/jsd_el2809_pub.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;
uint8_t      cmd_output[JSD_EL2809_NUM_CHANNELS] = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};

void telemetry_header() {
  int i;
  if (!file) {
    return;
  }
  for (i = 0; i < JSD_EL2809_NUM_CHANNELS; ++i) {
    fprintf(file, "EL2809_output_ch%d, ", i);
  }
  fprintf(file, "\n");
}

void telemetry_data(void* self) {
  assert(self);
  if (!file) {
    return;
  }

  single_device_server_t*   sds   = (single_device_server_t*)self;
  const jsd_el2809_state_t* state = jsd_el2809_get_state(sds->jsd, slave_id);

  int i;
  for (i = 0; i < JSD_EL2809_NUM_CHANNELS; ++i) {
    fprintf(file, "%u,", state->output[i]);
  }
  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  const jsd_el2809_state_t* state = jsd_el2809_get_state(sds->jsd, slave_id);

  MSG("Ch0: %u, Ch1: %u, Ch2: %u, Ch3: %u, Ch4: %u, Ch5: %u, Ch6: %u, Ch7: %u, Ch8: %u, "
      "Ch9: %u, Ch10: %u, Ch11: %u, Ch12: %u, Ch13: %u, Ch14: %u, Ch15: %u, ", 
      state->output[0], state->output[1], state->output[2], state->output[3], 
      state->output[4], state->output[5], state->output[6], state->output[7], 
      state->output[8], state->output[9], state->output[10], state->output[11], 
      state->output[12], state->output[13], state->output[14], state->output[15]);
}

void extract_data(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  jsd_el2809_process(sds->jsd, slave_id);
}

void command(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  cmd_output[0] ^= 0x01;
  cmd_output[1] ^= 0x01;
  cmd_output[2] ^= 0x01;
  cmd_output[3] ^= 0x01;

  jsd_el2809_write_all_channels(sds->jsd, slave_id, cmd_output);
}

int main(int argc, char const* argv[]) {
  if (argc != 4) {
    ERROR("Expecting exactly 3 arguments");
    MSG("Usage: jsd_el2809_test <ifname> <el2809_slave_index> <loop_freq_hz>");
    MSG("Example: $ jsd_el2809 eth0 5 500");
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
  my_config.device_type          = JSD_DEVICE_TYPE_EL2809;

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  sds_run(&sds, ifname, "/tmp/jsd_el2809.csv");

  return 0;
}
