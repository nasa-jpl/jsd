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
  fprintf(file, "cmd_position, ");
  fprintf(file, "cmd_velocity, ");
  fprintf(file, "cmd_current, ");
  fprintf(file, "cmd_max_current, ");
  fprintf(file, "cmd_ff_position, ");
  fprintf(file, "cmd_ff_velocity, ");
  fprintf(file, "cmd_ff_current, ");
  fprintf(file, "cmd_prof_velocity, ");
  fprintf(file, "cmd_prof_end_velocity, ");
  fprintf(file, "cmd_prof_accel, ");
  fprintf(file, "cmd_prof_decel, ");
  fprintf(file, "actual_state_machine_state, ");
  fprintf(file, "actual_mode_of_operation, ");
  fprintf(file, "sto_engaged, ");
  fprintf(file, "hall_state, ");
  fprintf(file, "motor_on, ");
  fprintf(file, "in_motion, ");
  fprintf(file, "servo_enabled, ");
  fprintf(file, "warning, ");
  fprintf(file, "target_reached, ");
  fprintf(file, "bus_voltage, ");
  fprintf(file, "analog_input_voltage, ");
  // Omitted digital inputs and digital output commands.
  fprintf(file, "drive_temperature, ");
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
  fprintf(file, "%i, ", state->cmd_position);
  fprintf(file, "%i, ", state->cmd_velocity);
  fprintf(file, "%lf, ", state->cmd_current);
  fprintf(file, "%lf, ", state->cmd_max_current);
  fprintf(file, "%i, ", state->cmd_ff_position);
  fprintf(file, "%i, ", state->cmd_ff_velocity);
  fprintf(file, "%lf, ", state->cmd_ff_current);
  fprintf(file, "%u, ", state->cmd_prof_velocity);
  fprintf(file, "%u, ", state->cmd_prof_end_velocity);
  fprintf(file, "%u, ", state->cmd_prof_accel);
  fprintf(file, "%u, ", state->cmd_prof_decel);
  fprintf(file, "%u, ", state->actual_state_machine_state);
  fprintf(file, "%u, ", state->actual_mode_of_operation);
  fprintf(file, "%u, ", state->sto_engaged);
  fprintf(file, "%u, ", state->hall_state);
  fprintf(file, "%u, ", state->motor_on);
  fprintf(file, "%u, ", state->in_motion);
  fprintf(file, "%u, ", state->servo_enabled);
  fprintf(file, "%u, ", state->warning);
  fprintf(file, "%u, ", state->target_reached);
  fprintf(file, "%lf, ", state->bus_voltage);
  fprintf(file, "%lf, ", state->analog_input_voltage);
  // Omitted digital inputs and digital output commands.
  fprintf(file, "%f, ", state->drive_temperature);
  fprintf(file, "%u, ", state->emcy_error_code);
  fprintf(file, "%lf, ", state->sil_input_r2_1);
  fprintf(file, "%lf, ", state->sil_output_r2_65);
  fprintf(file, "%lf, ", state->sil_input_r2_2);
  fprintf(file, "%lf, ", state->sil_output_r2_66);
  fprintf(file, "%lf, ", state->sil_input_r2_3);
  fprintf(file, "%lf, ", state->sil_output_r2_67);
  fprintf(file, "%lf, ", state->sil_input_r2_4);
  fprintf(file, "%lf, ", state->sil_output_r2_68);
  fprintf(file, "%lf, ", state->sil_input_r2_5);
  fprintf(file, "%lf, ", state->sil_output_r2_69);
  fprintf(file, "%lf, ", state->sil_input_r2_6);
  fprintf(file, "%lf, ", state->sil_output_r2_70);
  fprintf(file, "%lf, ", state->sil_input_r2_7);
  fprintf(file, "%lf, ", state->sil_output_r2_71);
  fprintf(file, "%lf, ", state->sil_input_r2_8);
  fprintf(file, "%lf, ", state->sil_output_r2_72);
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

    jsd_epd_set_sil_input_r2_1(sds->jsd, slave_id,
                               sil_input_r2_1_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2_2(sds->jsd, slave_id,
                               sil_input_r2_2_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2_3(sds->jsd, slave_id,
                               sil_input_r2_3_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2_4(sds->jsd, slave_id,
                               sil_input_r2_4_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2_5(sds->jsd, slave_id,
                               sil_input_r2_5_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2_6(sds->jsd, slave_id,
                               sil_input_r2_6_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2_7(sds->jsd, slave_id,
                               sil_input_r2_7_values[sil_input_value_idx]);
    jsd_epd_set_sil_input_r2_8(sds->jsd, slave_id,
                               sil_input_r2_8_values[sil_input_value_idx]);
  }

  ++iter;
}

int main(int argc, char* argv[]) {
  if (argc != 12) {
    ERROR("Expecting exactly 11 arguments");
    MSG("Usage: jsd_epd_sil_test <ifname> <epd_slave_index> <loop_freq_hz> "
        "<sil_input_r2_1> <sil_input_r2_2> <sil_input_r2_3> <sil_input_r2_4> "
        "<sil_input_r2_5> <sil_input_r2_6> <sil_input_r2_7> <sil_input_r2_8> ");
    MSG("Example: $ jsd_epd_sil_test eth0 2 100 11.11 22.22 33.33 44.44 55.55 "
        "66.66 77.77 88.88");
    return 0;
  }

  char* ifname         = strdup(argv[1]);
  slave_id             = atoi(argv[2]);
  int32_t loop_freq_hz = atoi(argv[3]);
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
  config.epd.max_motor_speed          = 1.0;
  config.epd.loop_period_ms           = 1000 / loop_freq_hz;
  config.epd.torque_slope             = 1e7;
  config.epd.max_profile_accel        = 1e6;
  config.epd.max_profile_decel        = 1e7;
  config.epd.velocity_tracking_error  = 1e8;
  config.epd.position_tracking_error  = 1e9;
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
  config.epd.smooth_factor        = 0;
  config.epd.ctrl_gain_scheduling_mode =
      JSD_ELMO_GAIN_SCHEDULING_MODE_PRELOADED;
  config.epd.use_sil = true;

  MSG("Configuring %i as loop_period_ms", config.epd.loop_period_ms);

  jsd_set_slave_config(sds.jsd, slave_id, config);

  server_startup_s = jsd_time_get_mono_time_sec();

  sds_run(&sds, ifname, "/tmp/jsd_epd_sil_test.csv");

  return 0;
}