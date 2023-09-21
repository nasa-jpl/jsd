#include <assert.h>
#include <math.h>
#include <string.h>

#include "jsd/jsd_epd_pub.h"
#include "jsd/jsd_time.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;
double       server_startup_s;
double       amplitude;
double       sine_freq;
uint8_t      enable_velocity_offset;
double       sil_input_r2_1_values[2];
double       sil_input_r2_2_values[2];
double       sil_input_r2_3_values[2];
double       sil_input_r2_4_values[2];
double       sil_input_r2_5_values[2];
double       sil_input_r2_6_values[2];
double       sil_input_r2_7_values[2];
double       sil_input_r2_8_values[2];
int32_t      sil_input_r1_1_values[2];
int32_t      sil_input_r1_2_values[2];
int32_t      sil_input_r1_3_values[2];

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
  fprintf(file, "sil_input_r2_1, ");
  fprintf(file, "sil_output_r2_65, ");
  fprintf(file, "sil_input_r2_2, ");
  fprintf(file, "sil_output_r2_66, ");
  fprintf(file, "sil_input_r2_3, ");
  fprintf(file, "sil_output_r2_67, ");
  fprintf(file, "sil_input_r2_4, ");
  fprintf(file, "sil_output_r2_68, ");
  fprintf(file, "sil_input_r2_5, ");
  fprintf(file, "sil_output_r2_69, ");
  fprintf(file, "sil_input_r2_6, ");
  fprintf(file, "sil_output_r2_70, ");
  fprintf(file, "sil_input_r2_7, ");
  fprintf(file, "sil_output_r2_71, ");
  fprintf(file, "sil_input_r2_8, ");
  fprintf(file, "sil_output_r2_72, ");
  fprintf(file, "sil_input_r1_1, ");
  fprintf(file, "sil_output_r1_129, ");
  fprintf(file, "sil_input_r1_2, ");
  fprintf(file, "sil_output_r1_130, ");
  fprintf(file, "sil_input_r1_3, ");
  fprintf(file, "sil_output_r1_131, ");
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

  single_device_server_t* sds   = (single_device_server_t*)self;
  const jsd_epd_state_t*  state = jsd_epd_get_state(sds->jsd, slave_id);

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
  fprintf(file, "%lf, ", jsd_epd_get_sil_input_r2(sds->jsd, slave_id, 1));
  fprintf(file, "%lf, ", jsd_epd_get_sil_output_r2(sds->jsd, slave_id, 65));
  fprintf(file, "%lf, ", jsd_epd_get_sil_input_r2(sds->jsd, slave_id, 2));
  fprintf(file, "%lf, ", jsd_epd_get_sil_output_r2(sds->jsd, slave_id, 66));
  fprintf(file, "%lf, ", jsd_epd_get_sil_input_r2(sds->jsd, slave_id, 3));
  fprintf(file, "%lf, ", jsd_epd_get_sil_output_r2(sds->jsd, slave_id, 67));
  fprintf(file, "%lf, ", jsd_epd_get_sil_input_r2(sds->jsd, slave_id, 4));
  fprintf(file, "%lf, ", jsd_epd_get_sil_output_r2(sds->jsd, slave_id, 68));
  fprintf(file, "%lf, ", jsd_epd_get_sil_input_r2(sds->jsd, slave_id, 5));
  fprintf(file, "%lf, ", jsd_epd_get_sil_output_r2(sds->jsd, slave_id, 69));
  fprintf(file, "%lf, ", jsd_epd_get_sil_input_r2(sds->jsd, slave_id, 6));
  fprintf(file, "%lf, ", jsd_epd_get_sil_output_r2(sds->jsd, slave_id, 70));
  fprintf(file, "%lf, ", jsd_epd_get_sil_input_r2(sds->jsd, slave_id, 7));
  fprintf(file, "%lf, ", jsd_epd_get_sil_output_r2(sds->jsd, slave_id, 71));
  fprintf(file, "%lf, ", jsd_epd_get_sil_input_r2(sds->jsd, slave_id, 8));
  fprintf(file, "%lf, ", jsd_epd_get_sil_output_r2(sds->jsd, slave_id, 72));
  fprintf(file, "%i, ", jsd_epd_get_sil_input_r1(sds->jsd, slave_id, 1));
  fprintf(file, "%i, ", jsd_epd_get_sil_output_r1(sds->jsd, slave_id, 129));
  fprintf(file, "%i, ", jsd_epd_get_sil_input_r1(sds->jsd, slave_id, 2));
  fprintf(file, "%i, ", jsd_epd_get_sil_output_r1(sds->jsd, slave_id, 130));
  fprintf(file, "%i, ", jsd_epd_get_sil_input_r1(sds->jsd, slave_id, 3));
  fprintf(file, "%i, ", jsd_epd_get_sil_output_r1(sds->jsd, slave_id, 131));
  fprintf(file, "%d, ", state->sil.sil_initialized);
  fprintf(file, "%d, ", state->sil.sil_running);
  fprintf(file, "%d, ", state->sil.sil_faulted);

  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) { (void)self; };

void extract_data(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  jsd_epd_process(sds->jsd, slave_id);
}

void command(void* self) {
  static int32_t iter                = 0;
  static size_t  sil_input_value_idx = 0;

  single_device_server_t* sds = (single_device_server_t*)self;

  double now_s = jsd_time_get_mono_time_sec();

  jsd_epd_read(sds->jsd, slave_id);
  const jsd_epd_state_t* state = jsd_epd_get_state(sds->jsd, slave_id);

  // Wait 2 seconds after server starts to issue first reset.
  if ((now_s - server_startup_s) < 2.0) {
    return;
  }

  // Reset whenever not in OPERATION ENABLED state.
  if (!state->servo_enabled) {
    MSG("Sending reset.");
    jsd_epd_reset(sds->jsd, slave_id);
    iter                = 0;
    sil_input_value_idx = 0;
    return;
  }

  // Alternate values of SIL inputs every 2 seconds.
  if (iter % (sds->loop_rate_hz * 2) == 0) {
    ++sil_input_value_idx;
    sil_input_value_idx %= 2;

    jsd_epd_set_sil_input_r2(sds->jsd, slave_id, 1,
                             sil_input_r2_1_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2(sds->jsd, slave_id, 2,
                             sil_input_r2_2_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2(sds->jsd, slave_id, 3,
                             sil_input_r2_3_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2(sds->jsd, slave_id, 4,
                             sil_input_r2_4_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2(sds->jsd, slave_id, 5,
                             sil_input_r2_5_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2(sds->jsd, slave_id, 6,
                             sil_input_r2_6_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2(sds->jsd, slave_id, 7,
                             sil_input_r2_7_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2(sds->jsd, slave_id, 8,
                             sil_input_r2_8_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r1(sds->jsd, slave_id, 1,
                             sil_input_r1_1_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r1(sds->jsd, slave_id, 2,
                             sil_input_r1_2_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r1(sds->jsd, slave_id, 3,
                             sil_input_r1_3_values[sil_input_value_idx]);
  }

  ++iter;
}

int main(int argc, char* argv[]) {
  if (argc != 15) {
    ERROR("Expecting exactly 14 arguments");
    MSG("Usage: jsd_epd_sil_test <ifname> <epd_slave_index> <loop_freq_hz> "
        "<sil_input_r2_1> <sil_input_r2_2> <sil_input_r2_3> <sil_input_r2_4> "
        "<sil_input_r2_5> <sil_input_r2_6> <sil_input_r2_7> <sil_input_r2_8> "
        "<sil_input_r1_1> <sil_input_r1_2> <sil_input_r1_3> ");
    MSG("Example: $ jsd_epd_sil_test eth0 2 100 11.11 22.22 33.33 44.44 55.55 "
        "66.66 77.77 88.88 1 2 3");
    return 0;
  }

  char* ifname             = strdup(argv[1]);
  slave_id                 = atoi(argv[2]);
  int32_t loop_freq_hz     = atoi(argv[3]);
  sil_input_r2_1_values[0] = atof(argv[4]);
  sil_input_r2_1_values[1] = sil_input_r2_1_values[0] + 100.0;
  sil_input_r2_2_values[0] = atof(argv[5]);
  sil_input_r2_2_values[1] = sil_input_r2_2_values[0] + 100.0;
  sil_input_r2_3_values[0] = atof(argv[6]);
  sil_input_r2_3_values[1] = sil_input_r2_3_values[0] + 100.0;
  sil_input_r2_4_values[0] = atof(argv[7]);
  sil_input_r2_4_values[1] = sil_input_r2_4_values[0] + 100.0;
  sil_input_r2_5_values[0] = atof(argv[8]);
  sil_input_r2_5_values[1] = sil_input_r2_5_values[0] + 100.0;
  sil_input_r2_6_values[0] = atof(argv[9]);
  sil_input_r2_6_values[1] = sil_input_r2_6_values[0] + 100.0;
  sil_input_r2_7_values[0] = atof(argv[10]);
  sil_input_r2_7_values[1] = sil_input_r2_7_values[0] + 100.0;
  sil_input_r2_8_values[0] = atof(argv[11]);
  sil_input_r2_8_values[1] = sil_input_r2_8_values[0] + 100.0;
  sil_input_r1_1_values[0] = atoi(argv[12]);
  sil_input_r1_1_values[1] = sil_input_r1_1_values[0] + 50;
  sil_input_r1_2_values[0] = atoi(argv[13]);
  sil_input_r1_2_values[1] = sil_input_r1_2_values[0] + 50;
  sil_input_r1_3_values[0] = atoi(argv[14]);
  sil_input_r1_3_values[1] = sil_input_r1_3_values[0] + 50;

  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using loop frequency of %i hz", loop_freq_hz);
  MSG("Using SIL input R2[1] alternating values {%lf, %lf}",
      sil_input_r2_1_values[0], sil_input_r2_1_values[1]);
  MSG("Using SIL input R2[2] alternating values {%lf, %lf}",
      sil_input_r2_2_values[0], sil_input_r2_2_values[1]);
  MSG("Using SIL input R2[3] alternating values {%lf, %lf}",
      sil_input_r2_3_values[0], sil_input_r2_3_values[1]);
  MSG("Using SIL input R2[4] alternating values {%lf, %lf}",
      sil_input_r2_4_values[0], sil_input_r2_4_values[1]);
  MSG("Using SIL input R2[5] alternating values {%lf, %lf}",
      sil_input_r2_5_values[0], sil_input_r2_5_values[1]);
  MSG("Using SIL input R2[6] alternating values {%lf, %lf}",
      sil_input_r2_6_values[0], sil_input_r2_6_values[1]);
  MSG("Using SIL input R2[7] alternating values {%lf, %lf}",
      sil_input_r2_7_values[0], sil_input_r2_7_values[1]);
  MSG("Using SIL input R2[8] alternating values {%lf, %lf}",
      sil_input_r2_8_values[0], sil_input_r2_8_values[1]);
  MSG("Using SIL input R1[1] alternating values {%i, %i}",
      sil_input_r1_1_values[0], sil_input_r1_1_values[1]);
  MSG("Using SIL input R1[2] alternating values {%i, %i}",
      sil_input_r1_2_values[0], sil_input_r1_2_values[1]);
  MSG("Using SIL input R1[3] alternating values {%i, %i}",
      sil_input_r1_3_values[0], sil_input_r1_3_values[1]);

  single_device_server_t sds;

  sds_set_telemetry_header_callback(&sds, telemetry_header);
  sds_set_telemetry_data_callback(&sds, telemetry_data);
  sds_set_print_info_callback(&sds, print_info);
  sds_set_extract_data_callback(&sds, extract_data);
  sds_set_command_callback(&sds, command);

  sds_setup(&sds, loop_freq_hz);

  jsd_slave_config_t config = {0};

  snprintf(config.name, JSD_NAME_LEN, "kukulkan");
  config.configuration_active         = true;
  config.product_code                 = JSD_EPD_PRODUCT_CODE_STD_FW;
  config.epd.peak_current_limit       = 0.25f;
  config.epd.peak_current_time        = 3.0f;
  config.epd.continuous_current_limit = 0.25f;
  config.epd.motor_stuck_current_level_pct =
      0.0f;  // Disable motor stuck protection.
  config.epd.motor_stuck_velocity_threshold = 0.0f;
  config.epd.motor_stuck_timeout            = 0.0f;
  config.epd.over_speed_threshold = 0.0;  // Disable over speed protection.
  config.epd.low_position_limit   = 0.0;
  config.epd.high_position_limit =
      config.epd.low_position_limit;  // Disable position limits protection.
  config.epd.brake_engage_msec    = BRAKE_TIME_MSEC;
  config.epd.brake_disengage_msec = BRAKE_TIME_MSEC;
  config.epd.use_sil              = true;
  config.epd.sil.inputs_r1_num    = 3;
  config.epd.sil.inputs_r2_num    = 8;
  config.epd.sil.outputs_r1_num   = 3;
  config.epd.sil.outputs_r2_num   = 8;

  jsd_set_slave_config(sds.jsd, slave_id, config);

  server_startup_s = jsd_time_get_mono_time_sec();

  sds_run(&sds, ifname, "/tmp/jsd_epd_sil_test.csv");

  return 0;
}