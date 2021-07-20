#include <assert.h>
#include <string.h>

#include "jsd/jsd_el3104_pub.h"
#include "jsd/jsd_el3104_types.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;

void telemetry_header() {
  int i = 0;
  if (!file) {
    return;
  }
  for (i = 0; i < JSD_EL3104_NUM_CHANNELS; ++i) {
    fprintf(file, "EL3602_ch%d_raw_value, EL3602_ch%d_volts, ", i, i);
    fprintf(file, "txPDO_state_ch%d, txPDO_toggle_ch%d, ", i, i);
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
  const jsd_el3104_state_t* state = jsd_el3104_get_state(sds->jsd, slave_id);

  int i = 0;
  for (i = 0; i < JSD_EL3104_NUM_CHANNELS; ++i) {
    fprintf(file, "%i, %lf,", state->adc_value[i], state->voltage[i]);
    fprintf(file, "%u, %u,", state->txPDO_state[i], state->txPDO_toggle[i]);
    fprintf(file, "%u, %u,", state->error[i], state->underrange[i]);
    fprintf(file, "%u,", state->overrange[i]);
  }
  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);

  single_device_server_t*   sds   = (single_device_server_t*)self;
  const jsd_el3104_state_t* state = jsd_el3104_get_state(sds->jsd, slave_id);
  MSG("Ch0: %f V,  Ch1: %f V, Ch2: %f V,  Ch3: %f V", 
    state->voltage[0], state->voltage[1], state->voltage[2], state->voltage[3]);
}

void extract_data(void* self) {
  single_device_server_t* sds = (single_device_server_t*)self;
  jsd_el3104_read(sds->jsd, slave_id);
}

void command(void* self) { (void)self; };

int main(int argc, char* argv[]) {
  if (argc != 4) {
    ERROR("Expecting exactly 3 arguments");
    MSG("Usage: jsd_el3104_test <ifname> <el3104_slave_index> <loop_freq_hz>");
    MSG("Example: $ jsd_el3104_test eth0 2 1000");
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
  my_config.product_code         = JSD_EL3104_PRODUCT_CODE;

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  sds_run(&sds, ifname, "/tmp/jsd_el3104.csv");

  return 0;
}
