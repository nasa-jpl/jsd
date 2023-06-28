#include <assert.h>
#include <string.h>

#include "jsd/jsd_ati_fts_pub.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;

void telemetry_header() {
  if (!file) {
    return;
  }

  fprintf(file, "Fx, Fy, Fz, Tx, Ty, Tz, active_fault, sample_counter,\n");
}

void telemetry_data(void* self) {
  assert(self);

  if (!file) {
    return;
  }

  single_device_server_t*    sds   = (single_device_server_t*)self;
  const jsd_ati_fts_state_t* state = jsd_ati_fts_get_state(sds->jsd, slave_id);

  fprintf(file, "%lf, %lf, %lf, %lf, %lf, %lf, %u, %u, \n", state->fx,
          state->fy, state->fz, state->tx, state->ty, state->tz,
          state->active_error, state->sample_counter);

  fflush(file);
}

void print_info(void* self) {
  assert(self);

  single_device_server_t*    sds   = (single_device_server_t*)self;
  const jsd_ati_fts_state_t* state = jsd_ati_fts_get_state(sds->jsd, slave_id);

  MSG("(%lf, %lf, %lf)  (%lf, %lf, %lf)  fault= %u counter=%u", state->fx,
      state->fy, state->fz, state->tx, state->ty, state->tz,
      state->active_error, state->sample_counter);
}

void extract_data(void* self) {
  single_device_server_t* sds = (single_device_server_t*)self;
  jsd_ati_fts_read(sds->jsd, slave_id);
}

void command(void* self) { (void)self; };

int main(int argc, char* argv[]) {
  if (argc != 5) {
    ERROR("Expecting exactly 4 arguments");
    MSG("Usage: jsd_ati_fts_test <ifname> <ati_fts_slave_index> <loop_freq_hz> "
        "<calibration>");
    MSG("Example: $ jsd_ati_fts_test eth0 0 1000 0");
    return 0;
  }

  char* ifname          = strdup(argv[1]);
  slave_id              = atoi(argv[2]);
  uint32_t loop_freq_hz = atoi(argv[3]);
  uint32_t calibration  = atoi(argv[4]);
  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using frequency of %i hz", loop_freq_hz);
  MSG("Using calibration of %u hz", calibration);

  single_device_server_t sds;

  sds_set_telemetry_header_callback(&sds, telemetry_header);
  sds_set_telemetry_data_callback(&sds, telemetry_data);
  sds_set_print_info_callback(&sds, print_info);
  sds_set_extract_data_callback(&sds, extract_data);
  sds_set_command_callback(&sds, command);

  sds_setup(&sds, loop_freq_hz);

  // set device configuration here
  jsd_slave_config_t my_config = {0};

  my_config.ati_fts.calibration = calibration;

  snprintf(my_config.name, JSD_NAME_LEN, "my_ati_fts");
  my_config.configuration_active = true;
  my_config.product_code         = JSD_ATI_FTS_PRODUCT_CODE;

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  sds_run(&sds, ifname, "/tmp/jsd_ati_fts.csv");

  return 0;
}
