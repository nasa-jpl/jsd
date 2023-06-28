#ifndef JSD_TEST_UTILS_H_
#define JSD_TEST_UTILS_H_

#include "jsd/jsd_pub.h"
#include "jsd/jsd_timer.h"

void on_sigint(int signal);

typedef struct {
  void (*telemetry_header)();
  void (*telemetry_data)(void* self);
  void (*print_info)(void* self);
  void (*extract_data)(void* self);
  void (*command)(void* self);

  jsd_timer_t* jsd_timer;
  uint16_t     loop_rate_hz;

  jsd_t* jsd;

} single_device_server_t;

void sds_set_telemetry_header_callback(single_device_server_t* self,
                                       void (*telemetry_header)());

void sds_set_telemetry_data_callback(single_device_server_t* self,
                                     void (*telemetry_data)(void*));

void sds_set_print_info_callback(single_device_server_t* self,
                                 void (*print_info)(void*));

void sds_set_extract_data_callback(single_device_server_t* self,
                                   void (*extract_data)(void*));

void sds_set_command_callback(single_device_server_t* self,
                              void (*command)(void*));

void sds_setup(single_device_server_t* self, uint16_t loop_rate_hz);

void sds_run(single_device_server_t* self, char* device_name, char* filename);
#endif
