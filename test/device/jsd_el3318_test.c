#include <assert.h>
#include <string.h>

#include "jsd/jsd_el3318_pub.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;

void telemetry_header() {
  int i;
  if (!file) {
    return;
  }
  for (i = 0; i < JSD_EL3318_NUM_CHANNELS; ++i) {
    fprintf(file, "EL3318_output_eu_ch%d, ", i);
    fprintf(file, "EL3318_raw_value_ch%d, ", i);
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
  const jsd_el3318_state_t* state = jsd_el3318_get_state(sds->jsd, slave_id);

  int i;
  for (i = 0; i < JSD_EL3318_NUM_CHANNELS; ++i) {
    fprintf(file, "%lf,", state->output_eu[i]);
    fprintf(file, "%i,", state->adc_value[i]);
    fprintf(file, "%u, %u,", state->txPDO_state[i], state->txPDO_toggle[i]);
    fprintf(file, "%u, %u,", state->error[i], state->underrange[i]);
    fprintf(file, "%u,", state->overrange[i]);
  }
  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);

  single_device_server_t* sds = (single_device_server_t*)self;
  jsd_el3318_read(sds->jsd, slave_id);
  const jsd_el3318_state_t* state = jsd_el3318_get_state(sds->jsd, slave_id);

  MSG("Ch0: %f,  Ch1: %f, Ch2: %f, Ch3: %f, Ch4: %f, Ch5: %f, Ch6: %f, Ch7: %f",
      state->output_eu[0], state->output_eu[1], state->output_eu[2],
      state->output_eu[3], state->output_eu[4], state->output_eu[5],
      state->output_eu[6], state->output_eu[7]);
}

void extract_data(void* self) { (void)self; }

void command(void* self) { (void)self; };

int main(int argc, char* argv[]) {
  if (argc != 4) {
    ERROR("Expecting exactly 3 arguments");
    MSG("Usage: jsd_el3318_test <ifname> <el3318_slave_index> <loop_freq_hz>");
    MSG("Example: $ jsd_el3318_test eth0 2 1000");
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

  my_config.el3318.element[0] = JSD_EL3318_ELEMENT_TYPE_K;
  my_config.el3318.element[1] = JSD_EL3318_ELEMENT_TYPE_T;
  my_config.el3318.element[2] = JSD_EL3318_ELEMENT_TYPE_K;
  my_config.el3318.element[3] = JSD_EL3318_ELEMENT_TYPE_K;
  my_config.el3318.element[4] = JSD_EL3318_ELEMENT_TYPE_K;
  my_config.el3318.element[5] = JSD_EL3318_ELEMENT_TYPE_K;
  my_config.el3318.element[6] = JSD_EL3318_ELEMENT_TYPE_K;
  my_config.el3318.element[7] = JSD_EL3318_ELEMENT_TYPE_K;

  my_config.el3318.filter[0] = JSD_BECKHOFF_FILTER_50HZ;
  my_config.el3318.filter[1] = JSD_BECKHOFF_FILTER_50HZ;
  my_config.el3318.filter[2] = JSD_BECKHOFF_FILTER_50HZ;
  my_config.el3318.filter[3] = JSD_BECKHOFF_FILTER_50HZ;
  my_config.el3318.filter[4] = JSD_BECKHOFF_FILTER_50HZ;
  my_config.el3318.filter[5] = JSD_BECKHOFF_FILTER_50HZ;
  my_config.el3318.filter[6] = JSD_BECKHOFF_FILTER_50HZ;
  my_config.el3318.filter[7] = JSD_BECKHOFF_FILTER_50HZ;

  my_config.el3318.presentation[0] = JSD_EL3318_PRESENTATION_SIGNED;
  my_config.el3318.presentation[1] = JSD_EL3318_PRESENTATION_SIGNED;
  my_config.el3318.presentation[2] = JSD_EL3318_PRESENTATION_SIGNED;
  my_config.el3318.presentation[3] = JSD_EL3318_PRESENTATION_SIGNED;
  my_config.el3318.presentation[4] = JSD_EL3318_PRESENTATION_SIGNED;
  my_config.el3318.presentation[5] = JSD_EL3318_PRESENTATION_SIGNED;
  my_config.el3318.presentation[6] = JSD_EL3318_PRESENTATION_SIGNED;
  my_config.el3318.presentation[7] = JSD_EL3318_PRESENTATION_SIGNED;

  snprintf(my_config.name, JSD_NAME_LEN, "yeti");
  my_config.configuration_active = true;
  my_config.product_code         = JSD_EL3318_PRODUCT_CODE;

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  sds_run(&sds, ifname, "/tmp/jsd_el3318.csv");

  return 0;
}
