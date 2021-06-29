#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint16_t     slave_id;

void telemetry_header() { MSG("you called telemetry_header"); }

void telemetry_data(void* self) {
  (void)self;
  MSG("you called telemetry_data");
}

void print_info(void* self) {
  (void)self;
  MSG("you called print_info");
}

void extract_data(void* self) {
  (void)self;
  MSG("you called extract_data");
}

void command(void* self) {
  (void)self;
  MSG("you called command");
}

int main(int argc, char* argv[]) {
  if (argc != 4) {
    ERROR("Expecting exactly 3 arguments");
    MSG("Usage: jsd_device_test_template <ifname> <slave_index> "
        "<loop_freq_hz>");
    MSG("Example: $ jsd_device_test_template eth0 2 1000");
    return 0;
  }

  char* ifname          = argv[1];
  slave_id              = atoi(argv[2]);
  uint32_t loop_freq_hz = atoi(argv[3]);
  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using frequency of %i hz \n", loop_freq_hz);

  single_device_server_t sds;

  sds_set_telemetry_header_callback(&sds, telemetry_header);
  sds_set_telemetry_data_callback(&sds, telemetry_data);
  sds_set_print_info_callback(&sds, print_info);
  sds_set_extract_data_callback(&sds, extract_data);
  sds_set_command_callback(&sds, command);

  sds_setup(&sds, loop_freq_hz);

  // set device configuration here e.g.
  // snprintf(sds.jsd->slave_config[slave_id].el3602.name, 64,
  // "my_el3602_device");
  // sds.jsd->slave_config[slave_id].el3602.range_factor[0] = EL3602_10V_RANGE;

  sds_run(&sds, ifname, "/tmp/jsd_test_utils_test.csv");

  return 0;
}
