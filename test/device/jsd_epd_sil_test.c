#include <assert.h>
#include <math.h>
#include <string.h>

#include "jsd/jsd_epd_sil_pub.h"
#include "jsd/jsd_sdo_pub.h"
#include "jsd/jsd_time.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;
double       server_startup_s;
double       amplitude;
double       sine_freq;
uint8_t      enable_velocity_offset;
int32_t      sil_r1_input_1_values[2];
int32_t      sil_r1_input_2_values[2];
int32_t      sil_r1_input_3_values[2];
int32_t      sil_r1_input_4_values[2];
int32_t      sil_r1_input_5_values[2];
double       sil_r2_input_1_values[2];
double       sil_r2_input_2_values[2];
double       sil_r2_input_3_values[2];
int32_t      sil_sdo_r1_12;
double       sil_sdo_r2_10;

int16_t BRAKE_TIME_MSEC = 100;

void telemetry_header() {
  if (!file) {
    return;
  }
  fprintf(file, "rel_time_s, ");
  fprintf(file, "cycle_period_s, ");

  fprintf(file, "actual_position, ");
  fprintf(file, "actual_velocity, ");
  fprintf(file, "actual_current, ");
  fprintf(file, "cmd_max_current, ");
  fprintf(file, "actual_state_machine_state, ");
  fprintf(file, "actual_mode_of_operation, ");
  fprintf(file, "sto_engaged, ");
  fprintf(file, "hall_state, ");
  fprintf(file, "motor_on, ");
  fprintf(file, "in_motion, ");
  fprintf(file, "servo_enabled, ");
  fprintf(file, "warning, ");
  fprintf(file, "target_reached, ");
  fprintf(file, "emcy_error_code, ");
  fprintf(file, "sil_r1_input_1, ");
  fprintf(file, "sil_r1_output_131, ");
  fprintf(file, "sil_r1_input_2, ");
  fprintf(file, "sil_r1_output_129, ");
  fprintf(file, "sil_r1_input_3, ");
  fprintf(file, "sil_r1_output_134, ");
  fprintf(file, "sil_r1_input_4, ");
  fprintf(file, "sil_r1_output_132, ");
  fprintf(file, "sil_r1_input_5, ");
  fprintf(file, "sil_r1_output_130, ");
  fprintf(file, "sil_r2_input_1, ");
  fprintf(file, "sil_r2_output_68, ");
  fprintf(file, "sil_r2_input_2, ");
  fprintf(file, "sil_r2_output_65, ");
  fprintf(file, "sil_r2_input_3, ");
  fprintf(file, "sil_r2_output_69, ");
  fprintf(file, "sil_r1_output_133, ");
  fprintf(file, "sil_r2_output_66, ");
  fprintf(file, "sil_r2_output_67, ");
  fprintf(file, "sil_sdo_r1_12, ");
  fprintf(file, "sil_sdo_r2_10, ");
  fprintf(file, "sil_initialized, ");
  fprintf(file, "sil_running, ");
  fprintf(file, "sil_faulted, ");

  fprintf(file, "\n");
}

void telemetry_data(void* self) {
  static double last_rel_time_s = 0.0;
  assert(self);
  if (!file) {
    return;
  }

  double now_s          = jsd_time_get_mono_time_sec();
  double rel_time_s     = now_s - server_startup_s;
  double cycle_period_s = rel_time_s - last_rel_time_s;
  last_rel_time_s       = rel_time_s;

  single_device_server_t*    sds   = (single_device_server_t*)self;
  const jsd_epd_sil_state_t* state = jsd_epd_sil_get_state(sds->jsd, slave_id);

  fprintf(file, "%lf, ", rel_time_s);
  fprintf(file, "%lf, ", cycle_period_s);

  fprintf(file, "%i, ", state->actual_position);
  fprintf(file, "%i, ", state->actual_velocity);
  fprintf(file, "%lf, ", state->actual_current);
  fprintf(file, "%lf, ", state->cmd_max_current);
  fprintf(file, "%u, ", state->actual_state_machine_state);
  fprintf(file, "%u, ", state->actual_mode_of_operation);
  fprintf(file, "%u, ", state->sto_engaged);
  fprintf(file, "%u, ", state->hall_state);
  fprintf(file, "%u, ", state->motor_on);
  fprintf(file, "%u, ", state->in_motion);
  fprintf(file, "%u, ", state->servo_enabled);
  fprintf(file, "%u, ", state->warning);
  fprintf(file, "%u, ", state->target_reached);
  fprintf(file, "%u, ", state->emcy_error_code);
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_input(sds->jsd, slave_id, 1));
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_output(sds->jsd, slave_id, 131));
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_input(sds->jsd, slave_id, 2));
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_output(sds->jsd, slave_id, 129));
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_input(sds->jsd, slave_id, 3));
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_output(sds->jsd, slave_id, 134));
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_input(sds->jsd, slave_id, 4));
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_output(sds->jsd, slave_id, 132));
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_input(sds->jsd, slave_id, 5));
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_output(sds->jsd, slave_id, 130));
  fprintf(file, "%lf, ", jsd_epd_sil_get_sil_r2_input(sds->jsd, slave_id, 1));
  fprintf(file, "%lf, ", jsd_epd_sil_get_sil_r2_output(sds->jsd, slave_id, 68));
  fprintf(file, "%lf, ", jsd_epd_sil_get_sil_r2_input(sds->jsd, slave_id, 2));
  fprintf(file, "%lf, ", jsd_epd_sil_get_sil_r2_output(sds->jsd, slave_id, 65));
  fprintf(file, "%lf, ", jsd_epd_sil_get_sil_r2_input(sds->jsd, slave_id, 3));
  fprintf(file, "%lf, ", jsd_epd_sil_get_sil_r2_output(sds->jsd, slave_id, 69));
  fprintf(file, "%i, ", jsd_epd_sil_get_sil_r1_output(sds->jsd, slave_id, 133));
  fprintf(file, "%lf, ", jsd_epd_sil_get_sil_r2_output(sds->jsd, slave_id, 66));
  fprintf(file, "%lf, ", jsd_epd_sil_get_sil_r2_output(sds->jsd, slave_id, 67));
  fprintf(file, "%i, ", sil_sdo_r1_12);
  fprintf(file, "%lf, ", sil_sdo_r2_10);
  fprintf(file, "%d, ", state->sil_initialized);
  fprintf(file, "%d, ", state->sil_running);
  fprintf(file, "%d, ", state->sil_faulted);

  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) { (void)self; };

void extract_data(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  jsd_epd_sil_process(sds->jsd, slave_id);
}

void command(void* self) {
  static int32_t  iter                = 0;
  static size_t   sil_input_value_idx = 0;
  static bool     first_sdos_sent     = false;
  static uint16_t sdo_app_id          = 0;
  static int32_t  sdo_r1_value        = -6;
  static double   sdo_r2_value        = -20.0;

  single_device_server_t* sds = (single_device_server_t*)self;

  double now_s = jsd_time_get_mono_time_sec();

  jsd_epd_sil_read(sds->jsd, slave_id);
  const jsd_epd_sil_state_t* state = jsd_epd_sil_get_state(sds->jsd, slave_id);

  // Wait 2 seconds after server starts to issue first reset.
  if ((now_s - server_startup_s) < 2.0) {
    return;
  }

  // Reset whenever not in OPERATION ENABLED state.
  if (!state->servo_enabled) {
    MSG("Sending reset.");
    jsd_epd_sil_reset(sds->jsd, slave_id);
    iter                = 0;
    sil_input_value_idx = 0;
    return;
  }

  // Alternate values of SIL inputs every 2 seconds.
  if (iter % (sds->loop_rate_hz * 2) == 0) {
    ++sil_input_value_idx;
    sil_input_value_idx %= 2;

    jsd_epd_sil_set_sil_r1_input(sds->jsd, slave_id, 1,
                                 sil_r1_input_1_values[sil_input_value_idx]);
    jsd_epd_sil_set_sil_r1_input(sds->jsd, slave_id, 2,
                                 sil_r1_input_2_values[sil_input_value_idx]);
    jsd_epd_sil_set_sil_r1_input(sds->jsd, slave_id, 3,
                                 sil_r1_input_3_values[sil_input_value_idx]);
    jsd_epd_sil_set_sil_r1_input(sds->jsd, slave_id, 4,
                                 sil_r1_input_4_values[sil_input_value_idx]);
    jsd_epd_sil_set_sil_r1_input(sds->jsd, slave_id, 5,
                                 sil_r1_input_5_values[sil_input_value_idx]);
    jsd_epd_sil_set_sil_r2_input(sds->jsd, slave_id, 1,
                                 sil_r2_input_1_values[sil_input_value_idx]);
    jsd_epd_sil_set_sil_r2_input(sds->jsd, slave_id, 2,
                                 sil_r2_input_2_values[sil_input_value_idx]);
    jsd_epd_sil_set_sil_r2_input(sds->jsd, slave_id, 3,
                                 sil_r2_input_3_values[sil_input_value_idx]);

    // Write SDO
    jsd_epd_sil_async_sdo_set_sil_r1(sds->jsd, slave_id, 12, sdo_r1_value++,
                                     sdo_app_id++);
    jsd_epd_sil_async_sdo_set_sil_r2(sds->jsd, slave_id, 10, sdo_r2_value++,
                                     sdo_app_id++);
    // Read SDO
    uint16_t sdo_get_r1_app_id_prev = sdo_app_id - 4;
    jsd_epd_sil_async_sdo_get_sil_r1(sds->jsd, slave_id, 12, sdo_app_id++);
    uint16_t sdo_get_r2_app_id_prev = sdo_app_id - 4;
    jsd_epd_sil_async_sdo_get_sil_r2(sds->jsd, slave_id, 10, sdo_app_id++);

    if (first_sdos_sent) {
      jsd_sdo_req_t response;
      while (jsd_sdo_pop_response_queue(sds->jsd, &response)) {
        if (response.app_id == sdo_get_r1_app_id_prev) {
          if (response.success) {
            sil_sdo_r1_12 = response.data.as_i32;
          } else {
            ERROR(
                "R1[12] SDO read request was received but it was "
                "unsuccessful.");
          }
        } else if (response.app_id == sdo_get_r2_app_id_prev) {
          if (response.success) {
            sil_sdo_r2_10 = response.data.as_double;
          } else {
            ERROR(
                "R2[10] SDO read request was received but it was "
                "unsuccessful.");
          }
        }
      }
    }
    first_sdos_sent = true;
  }

  ++iter;
}

int main(int argc, char* argv[]) {
  if (argc != 12) {
    ERROR("Expecting exactly 11 arguments");
    MSG("Usage: jsd_epd_sil_test <ifname> <epd_slave_index> <loop_freq_hz> "
        "<sil_r1_input_1> <sil_r1_input_2> <sil_input_r1_3> <sil_input_r1_4> "
        "<sil_r1_input_5> <sil_r2_input_1> <sil_r2_input_2> <sil_input_r2_3>");
    MSG("Example: $ jsd_epd_sil_test eth0 2 100  1 2 3 4 5 11.11 22.22 33.33");
    return 0;
  }

  char* ifname             = strdup(argv[1]);
  slave_id                 = atoi(argv[2]);
  int32_t loop_freq_hz     = atoi(argv[3]);
  sil_r1_input_1_values[0] = atoi(argv[4]);
  sil_r1_input_1_values[1] = sil_r1_input_1_values[0] + 50;
  sil_r1_input_2_values[0] = atoi(argv[5]);
  sil_r1_input_2_values[1] = sil_r1_input_2_values[0] + 50;
  sil_r1_input_3_values[0] = atoi(argv[6]);
  sil_r1_input_3_values[1] = sil_r1_input_3_values[0] + 50;
  sil_r1_input_4_values[0] = atoi(argv[7]);
  sil_r1_input_4_values[1] = sil_r1_input_4_values[0] + 50;
  sil_r1_input_5_values[0] = atoi(argv[8]);
  sil_r1_input_5_values[1] = sil_r1_input_5_values[0] + 50;
  sil_r2_input_1_values[0] = atof(argv[9]);
  sil_r2_input_1_values[1] = sil_r2_input_1_values[0] + 100.0;
  sil_r2_input_2_values[0] = atof(argv[10]);
  sil_r2_input_2_values[1] = sil_r2_input_2_values[0] + 100.0;
  sil_r2_input_3_values[0] = atof(argv[11]);
  sil_r2_input_3_values[1] = sil_r2_input_3_values[0] + 100.0;

  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using loop frequency of %i hz", loop_freq_hz);
  MSG("Using SIL input R1[1] alternating values {%i, %i}",
      sil_r1_input_1_values[0], sil_r1_input_1_values[1]);
  MSG("Using SIL input R1[2] alternating values {%i, %i}",
      sil_r1_input_2_values[0], sil_r1_input_2_values[1]);
  MSG("Using SIL input R1[3] alternating values {%i, %i}",
      sil_r1_input_3_values[0], sil_r1_input_3_values[1]);
  MSG("Using SIL input R1[4] alternating values {%i, %i}",
      sil_r1_input_4_values[0], sil_r1_input_4_values[1]);
  MSG("Using SIL input R1[5] alternating values {%i, %i}",
      sil_r1_input_5_values[0], sil_r1_input_5_values[1]);
  MSG("Using SIL input R2[1] alternating values {%lf, %lf}",
      sil_r2_input_1_values[0], sil_r2_input_1_values[1]);
  MSG("Using SIL input R2[2] alternating values {%lf, %lf}",
      sil_r2_input_2_values[0], sil_r2_input_2_values[1]);
  MSG("Using SIL input R2[3] alternating values {%lf, %lf}",
      sil_r2_input_3_values[0], sil_r2_input_3_values[1]);

  single_device_server_t sds;

  sds_set_telemetry_header_callback(&sds, telemetry_header);
  sds_set_telemetry_data_callback(&sds, telemetry_data);
  sds_set_print_info_callback(&sds, print_info);
  sds_set_extract_data_callback(&sds, extract_data);
  sds_set_command_callback(&sds, command);

  sds_setup(&sds, loop_freq_hz);

  jsd_slave_config_t config = {0};

  snprintf(config.name, JSD_NAME_LEN, "kukulkan");
  config.configuration_active             = true;
  config.driver_type                      = JSD_DRIVER_TYPE_EPD_SIL;
  config.epd_sil.peak_current_limit       = 0.25f;
  config.epd_sil.peak_current_time        = 3.0f;
  config.epd_sil.continuous_current_limit = 0.25f;
  config.epd_sil.motor_stuck_current_level_pct =
      0.0f;  // Disable motor stuck protection.
  config.epd_sil.motor_stuck_velocity_threshold = 0.0f;
  config.epd_sil.motor_stuck_timeout            = 0.0f;
  config.epd_sil.over_speed_threshold = 0.0;  // Disable over speed protection.
  config.epd_sil.low_position_limit   = 0.0;
  config.epd_sil.high_position_limit =
      config.epd_sil.low_position_limit;  // Disable position limits protection.
  config.epd_sil.brake_engage_msec    = BRAKE_TIME_MSEC;
  config.epd_sil.brake_disengage_msec = BRAKE_TIME_MSEC;
  config.epd_sil.sil_r1_inputs_num    = 5;
  config.epd_sil.sil_r2_inputs_num    = 3;
  config.epd_sil.sil_r1_outputs_num   = 6;
  config.epd_sil.sil_r2_outputs_num   = 5;

  jsd_set_slave_config(sds.jsd, slave_id, config);

  server_startup_s = jsd_time_get_mono_time_sec();

  sds_run(&sds, ifname, "/tmp/jsd_epd_sil_test.csv");

  return 0;
}