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
int32_t      target_velocities[2] = {0, 0};

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
  static int32_t iter    = 0;
  static size_t  vel_idx = 0;

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
    iter = 0;
    return;
  }

  // Change target velocity every 5 seconds.
  if (iter % (sds->loop_rate_hz * 5) == 0) {
    ++vel_idx;
    vel_idx %= 2;
  }

  jsd_epd_motion_command_csv_t csv;
  csv.target_velocity    = target_velocities[vel_idx];
  csv.velocity_offset    = 0;
  csv.torque_offset_amps = 0.0;

  jsd_epd_set_motion_command_csv(sds->jsd, slave_id, csv);
  ++iter;
}

int main(int argc, char* argv[]) {
  if (argc != 9) {
    ERROR("Expecting exactly 8 arguments");
    MSG("Usage: jsd_epd_csv_test <ifname> <epd_slave_index> <loop_freq_hz> "
        "<target_velocity> <over_speed_threshold> <peak_current_amps> "
        "<continuous_current_amps> <max_motor_speed> ");
    MSG("Example: $ jsd_epd_csv_test eth0 2 100 25000 35000 1.0 0.45 50000");
    return 0;
  }

  char* ifname                 = strdup(argv[1]);
  slave_id                     = atoi(argv[2]);
  int32_t loop_freq_hz         = atoi(argv[3]);
  int32_t target_velocity      = atoi(argv[4]);
  double  over_speed_threshold = atof(argv[5]);
  float   peak_current         = atof(argv[6]);
  float   continuous_current   = atof(argv[7]);
  double  max_motor_speed      = atof(argv[8]);

  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using loop frequency of %i hz", loop_freq_hz);
  MSG("Using target velocity of %i cnts/sec", target_velocity);
  MSG("Using over speed threshold of %lf cnts/sec", over_speed_threshold);
  MSG("Using peak current of %f A", peak_current);
  MSG("Using continuous current of %f A", continuous_current);
  MSG("Using max_motor_speed of %lf cnts/sec", max_motor_speed);

  target_velocities[0] = target_velocity;
  target_velocities[1] = target_velocity / 2;

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
  config.product_code                 = JSD_EPD_PRODUCT_CODE;
  config.epd.max_motor_speed          = max_motor_speed;
  config.epd.loop_period_ms           = 1000 / loop_freq_hz;
  config.epd.torque_slope             = 1e7;
  config.epd.max_profile_accel        = 1e6;
  config.epd.max_profile_decel        = 1e7;
  config.epd.velocity_tracking_error  = 1e8;
  config.epd.position_tracking_error  = 1e9;
  config.epd.peak_current_limit       = peak_current;
  config.epd.peak_current_time        = 3.0f;
  config.epd.continuous_current_limit = continuous_current;
  config.epd.motor_stuck_current_level_pct =
      0.0f;  // Disable motor stuck protection.
  config.epd.motor_stuck_velocity_threshold = 0.0f;
  config.epd.motor_stuck_timeout            = 0.0f;
  config.epd.over_speed_threshold           = over_speed_threshold;
  config.epd.low_position_limit             = 0.0;
  config.epd.high_position_limit =
      config.epd.low_position_limit;  // Disable position limits protection.
  config.epd.brake_engage_msec    = BRAKE_TIME_MSEC;
  config.epd.brake_disengage_msec = BRAKE_TIME_MSEC;
  config.epd.smooth_factor        = 0;

  MSG("Configuring %i as loop_period_ms", config.epd.loop_period_ms);

  jsd_set_slave_config(sds.jsd, slave_id, config);

  server_startup_s = jsd_time_get_mono_time_sec();

  sds_run(&sds, ifname, "/tmp/jsd_epd_csv_test.csv");

  return 0;
}