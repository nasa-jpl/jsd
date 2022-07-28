#include <assert.h>
#include <string.h>

#include "jsd/jsd_ild1900_pub.h"
#include "jsd/jsd_ild1900_types.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;

void telemetry_header() {
  if (!file) {
    return;
  }
  fprintf(file, "ILD1900_distance_m, ILD1900_linearized_distance_raw, ");
  fprintf(file, "unlinearized_center_of_gravity_percent, intensity_percent, ");
  fprintf(file,
          "peak_distance_raw, timestamp_ns, counter, sensor_status, error");
  fprintf(file, "\n");
}

void telemetry_data(void* self) {
  assert(self);

  if (!file) {
    return;
  }

  single_device_server_t*    sds   = (single_device_server_t*)self;
  const jsd_ild1900_state_t* state = jsd_ild1900_get_state(sds->jsd, slave_id);

  fprintf(file, "%lf, %u, ", state->distance, state->linearized_distance_raw);
  fprintf(file, "%lf, ", state->unlinearized_center_of_gravity);
  fprintf(file, "%lf, %u, %u, ", state->intensity, state->peak_distance,
          state->timestamp);
  fprintf(file, "%u, %u, ", state->counter, state->sensor_status);
  fprintf(file, "%i", state->error);
  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);

  single_device_server_t*    sds   = (single_device_server_t*)self;
  const jsd_ild1900_state_t* state = jsd_ild1900_get_state(sds->jsd, slave_id);
  MSG("Distance: %lf m, Distance raw: %d, Error: %i", state->distance,
      state->peak_distance, state->error);
}

void extract_data(void* self) {
  assert(self);

  single_device_server_t* sds = (single_device_server_t*)self;
  jsd_ild1900_read(sds->jsd, slave_id);
}

void command(void* self) { (void)self; };

int main(int argc, char* argv[]) {
  if (argc != 4) {
    ERROR("Expecting exactly 3 arguments");
    MSG("Usage: jsd_ild1900_test <ifname> <ild1900_slave_index> "
        "<loop_freq_hz>");
    MSG("Example: $ jsd_ild1900_test eth0 2 1000");
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

  jsd_slave_config_t my_config = {0};

  snprintf(my_config.name, JSD_NAME_LEN, "huehueteotl");
  my_config.configuration_active     = true;
  my_config.product_code             = JSD_ILD1900_PRODUCT_CODE;
  my_config.ild1900.model            = JSD_ILD1900_MODEL_100;
  my_config.ild1900.measuring_rate   = 250.0;
  my_config.ild1900.averaging_type   = JSD_ILD1900_AVERAGING_MOVING;
  my_config.ild1900.averaging_number = 16;
  my_config.ild1900.peak_selection   = JSD_ILD1900_PEAK_SELECTION_HIGHEST;
  my_config.ild1900.exposure_mode    = JSD_ILD1900_EXPOSURE_MODE_STANDARD;

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  sds_run(&sds, ifname, "/tmp/jsd_ild1900.csv");

  free(ifname);

  return 0;
}