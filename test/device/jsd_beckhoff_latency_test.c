#include <assert.h>
#include <string.h>

#include "jsd/jsd_el2124_pub.h"
#include "jsd/jsd_el3602_pub.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      el2124_slave_id;
uint8_t      el3602_slave_id;
uint8_t      cmd_output[JSD_EL2124_NUM_CHANNELS] = {0, 0, 0, 0};
uint32_t     cmd_count                           = 0;

void telemetry_header() {
  int i;
  if (!file) {
    return;
  }
  for (i = 0; i < JSD_EL2124_NUM_CHANNELS; ++i) {
    fprintf(file, "EL2124_output_ch%d, ", i);
  }
  for (i = 0; i < JSD_EL3602_NUM_CHANNELS; ++i) {
    fprintf(file, "EL3602_ch%d_raw_value, EL3602_ch%d_volts, ", i, i);
    fprintf(file, "limit1_ch%d, limit2_ch%d, ", i, i);
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

  int i;
  for (i = 0; i < JSD_EL2124_NUM_CHANNELS; ++i) {
    fprintf(file, "%u,", cmd_output[i]);
  }

  single_device_server_t*   sds = (single_device_server_t*)self;
  const jsd_el3602_state_t* state =
      jsd_el3602_get_state(sds->jsd, el3602_slave_id);

  for (i = 0; i < JSD_EL3602_NUM_CHANNELS; ++i) {
    fprintf(file, "%u, %lf,", state->adc_value[i], state->voltage[i]);
    fprintf(file, "%u, %u,", state->limit1[i], state->limit2[i]);
    fprintf(file, "%u, %u,", state->txPDO_state[i], state->txPDO_toggle[i]);
    fprintf(file, "%u, %u,", state->error[i], state->underrange[i]);
    fprintf(file, "%u,", state->overrange[i]);
  }
  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);
  single_device_server_t*   sds = (single_device_server_t*)self;
  const jsd_el3602_state_t* state =
      jsd_el3602_get_state(sds->jsd, el3602_slave_id);

  MSG("EL2124 Ch0: %u, EL3602 Ch0: %f", cmd_output[0], state->voltage[0]);
}

void extract_data(void* self) {
  single_device_server_t* sds = (single_device_server_t*)self;
  jsd_el3602_read(sds->jsd, el3602_slave_id);
  jsd_el2124_process(sds->jsd, el2124_slave_id);
}

void command(void* self) {
  if (0 == cmd_count++ % 100) {
    single_device_server_t* ssd = (single_device_server_t*)self;

    cmd_output[0] ^= 0x01;

    jsd_el2124_write_all_channels(ssd->jsd, el2124_slave_id, cmd_output);
  }
};

int main(int argc, char* argv[]) {
  if (argc != 5) {
    ERROR("Expecting exactly 4 arguments");
    MSG("Usage: jsd_beckhoff_latency_test <ifname> <el3602_slave_index> \
        <el2124_slave_index> <loop_freq_hz>");
    MSG("Example: $ jsd_beckhoff_latency_test eth0 2 5 1000");
    return 0;
  }

  char* ifname          = argv[1];
  el3602_slave_id       = atoi(argv[2]);
  el2124_slave_id       = atoi(argv[3]);
  uint32_t loop_freq_hz = atoi(argv[4]);
  MSG("Configuring device %s", ifname);
  MSG("Using EL3602 at %d and EL2124 at %d", el3602_slave_id, el2124_slave_id);
  MSG("Using frequency of %i hz", loop_freq_hz);

  single_device_server_t sds;

  sds_set_telemetry_header_callback(&sds, telemetry_header);
  sds_set_telemetry_data_callback(&sds, telemetry_data);
  sds_set_print_info_callback(&sds, print_info);
  sds_set_extract_data_callback(&sds, extract_data);
  sds_set_command_callback(&sds, command);

  sds_setup(&sds, loop_freq_hz);

  // set device configuration here
  jsd_slave_config_t slave_config = {0};

  slave_config.el3602.range[0]  = JSD_EL3602_RANGE_5V;
  slave_config.el3602.range[1]  = JSD_EL3602_RANGE_5V;
  slave_config.el3602.filter[0] = JSD_BECKHOFF_FILTER_30000HZ;
  slave_config.el3602.filter[1] = JSD_BECKHOFF_FILTER_30000HZ;
  snprintf(slave_config.name, JSD_NAME_LEN, "unicorn");
  slave_config.configuration_active = true;
  slave_config.product_code         = JSD_EL3602_PRODUCT_CODE;

  jsd_set_slave_config(sds.jsd, el3602_slave_id, slave_config);

  jsd_slave_config_t slave_config2   = {0};
  slave_config2.configuration_active = true;
  snprintf(slave_config2.name, JSD_NAME_LEN, "bigfoot");
  slave_config2.product_code = JSD_EL2124_PRODUCT_CODE;

  jsd_set_slave_config(sds.jsd, el2124_slave_id, slave_config2);

  sds_run(&sds, ifname, "/tmp/jsd_beckhoff_latency.csv");

  return 0;
}
