#include "jsd_test_utils.h"

#include <assert.h>
#include <signal.h>
#include <time.h>

#define NSEC_PER_SEC (1000000000L)

bool   quit      = false;
FILE*  file      = NULL;
double last_time = 0;

void on_sigint(int signal) {
  (void)signal;
  MSG("Received SIGINT stopping JSD now");
  quit = true;
}

void sds_set_telemetry_header_callback(single_device_server_t* self,
                                       void (*telemetry_header)()) {
  assert(self);
  assert(telemetry_header);
  MSG_DEBUG("Assigned telemetry_header");
  self->telemetry_header = telemetry_header;
}

void sds_set_telemetry_data_callback(single_device_server_t* self,
                                     void (*telemetry_data)(void*)) {
  assert(self);
  assert(telemetry_data);
  MSG_DEBUG("Assigned telemetry_data");
  self->telemetry_data = telemetry_data;
}

void sds_set_print_info_callback(single_device_server_t* self,
                                 void (*print_info)(void*)) {
  assert(self);
  assert(print_info);
  MSG_DEBUG("Assigned print_info");
  self->print_info = print_info;
}

void sds_set_extract_data_callback(single_device_server_t* self,
                                   void (*extract_data)(void*)) {
  assert(self);
  assert(extract_data);
  MSG_DEBUG("Assigned extract_data");
  self->extract_data = extract_data;
}

void sds_set_command_callback(single_device_server_t* self,
                              void (*command)(void*)) {
  assert(self);
  assert(command);
  MSG_DEBUG("Assigned command");
  self->command = command;
}

void sds_setup(single_device_server_t* self, uint16_t loop_rate_hz) {
  assert(self);
  MSG_DEBUG("SDS_SETUP Begin");

  self->loop_rate_hz = loop_rate_hz;

  // setup jsd_timer without real-time settings
  self->jsd_timer = jsd_timer_alloc();

  uint32_t loop_period_ns = 1e9 / loop_rate_hz;
  MSG("jsd_timer using period %u nsec", loop_period_ns);

  // assert(0 == jsd_timer_init_ex(self->jsd_timer, loop_period_ns,
  //                              JPL_TIMER_ANY_CPU, false, false));
  assert(0 ==
         jsd_timer_init_ex(self->jsd_timer, loop_period_ns, 0, true, true));

  if (signal(SIGINT, on_sigint) == SIG_ERR) {
    ERROR("Could not register SIGINT handler function");
    return;
  }

  self->jsd = jsd_alloc();
  MSG("jsd address: %p", self->jsd);

  MSG_DEBUG("SDS_SETUP End");
}

void sds_run(single_device_server_t* self, char* device_name, char* filename) {
  assert(self);
  assert(self->telemetry_header);
  assert(self->telemetry_data);
  assert(self->print_info);
  assert(self->extract_data);
  assert(self->command);

  MSG_DEBUG("SDS_RUN Begin");

  file = fopen(filename, "w");
  if (!file) {
    ERROR("Could not open file for writing: %s", filename);
  }

  self->telemetry_header();

  uint32_t sds_iter = 0;

  if (!jsd_init(self->jsd, device_name, 1)) {
    ERROR("Could not init jsd");
    return;
  }

  MSG_DEBUG("Starting the loop now");

  while (!quit) {
    jsd_read(self->jsd, EC_TIMEOUTRET);

    self->command(self);

    self->extract_data(self);

    // MSG("iter: %u\n", sds_iter);
    if (sds_iter % self->loop_rate_hz == 0) {
      self->print_info(self);
    }

    jsd_write(self->jsd);

    self->telemetry_data(self);

    jsd_timer_process(self->jsd_timer);
    sds_iter++;
  }

  MSG_DEBUG("Closing now");
  fclose(file);
  jsd_free(self->jsd);
  MSG_DEBUG("jsd freed");
  MSG_DEBUG("SDS_RUN End");
}
