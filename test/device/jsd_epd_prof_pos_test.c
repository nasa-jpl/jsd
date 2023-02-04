#include <assert.h>
#include <string.h>

#include "jsd/jsd_epd_pub.h"
#include "jsd/jsd_time.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;
double       server_startup_s;
int32_t      initial_target_pos;
int32_t      displ_from_initial_target;
uint32_t     profile_velocity;
uint32_t     profile_accel;
uint32_t     profile_decel;

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

  // TODO(dloret): Delete these debugging fields.
  fprintf(file, "controlword, ");
  fprintf(file, "prof_pos_new_setpoint, ");
  fprintf(file, "prof_pos_change_setpoint, ");
  fprintf(file, "prof_pos_relative, ");

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

  // TODO(dloret): Delete these debugging fields.
  jsd_epd_private_state_t* private_state =
      &sds->jsd->slave_states[slave_id].epd;
  fprintf(file, "%u, ", private_state->rxpdo.controlword);
  fprintf(file, "%u, ", ((private_state->rxpdo.controlword >> 4) & 0x01));
  fprintf(file, "%u, ", ((private_state->rxpdo.controlword >> 5) & 0x01));
  fprintf(file, "%u, ", ((private_state->rxpdo.controlword >> 6) & 0x01));

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
  static bool    abs_cmd_sent       = false;
  static int32_t iter               = 0;
  static int32_t abs_cmd_iter_stamp = 0;

  single_device_server_t* sds = (single_device_server_t*)self;

  double now_s = jsd_time_get_mono_time_sec();

  jsd_epd_read(sds->jsd, slave_id);
  const jsd_epd_state_t* state = jsd_epd_get_state(sds->jsd, slave_id);

  // Wait 4 seconds after server starts to issue first reset.
  if ((now_s - server_startup_s) < 4.0) {
    return;
  }

  // Reset whenever not in OPERATION ENABLED state.
  if (!state->servo_enabled) {
    MSG("Sending reset.");
    jsd_epd_reset(sds->jsd, slave_id);
    iter = 0;
    return;
  }

  if (!abs_cmd_sent) {
    jsd_epd_motion_command_prof_pos_t cmd;
    cmd.target_position  = initial_target_pos;
    cmd.profile_velocity = profile_velocity;
    cmd.end_velocity     = 0;
    cmd.profile_accel    = profile_accel;
    cmd.profile_decel    = profile_decel;
    cmd.relative         = 0;

    jsd_epd_set_motion_command_prof_pos(sds->jsd, slave_id, cmd);

    abs_cmd_sent       = true;
    abs_cmd_iter_stamp = iter;
  }

  // Wait a few cycles after the absolute profiled position command is sent
  // before checking whether the target has been reached. This is necessary
  // because target_reached is 1 when entering the drive's Enabled state until
  // a new command is issued.
  // Afterwards, command a displacement every 10 seconds.
  if (((iter - abs_cmd_iter_stamp) > 10) &&
      (iter % (sds->loop_rate_hz * 10) == 0) && (state->target_reached == 1)) {
    jsd_epd_motion_command_prof_pos_t cmd;
    displ_from_initial_target *= -1;
    cmd.target_position  = displ_from_initial_target;
    cmd.profile_velocity = profile_velocity;
    cmd.end_velocity     = 0;
    cmd.profile_accel    = profile_accel;
    cmd.profile_decel    = profile_decel;
    cmd.relative         = 1;

    jsd_epd_set_motion_command_prof_pos(sds->jsd, slave_id, cmd);
  }

  ++iter;
}

int main(int argc, char* argv[]) {
  if (argc != 12) {
    ERROR("Expecting exactly 11 arguments");
    MSG("Usage: jsd_epd_prof_pos_test <ifname> <epd_slave_index> "
        "<loop_freq_hz> <target_position> <displacement_from_target> "
        "<profile_velocity> <profile_accel> <profile_decel> "
        "<peak_current_amps> <continuous_current_amps> <max_motor_speed> ");
    MSG("Example: $ jsd_epd_prof_pos_test eth0 2 100 25000 1000 20000 5000 "
        "10000 0.50 0.25 50000");
    return 0;
  }

  if (atoi(argv[6]) < 0 || atoi(argv[7]) < 0 || atoi(argv[8]) < 0) {
    ERROR(
        "Profile velocity, profile acceleration, and profile deceleration must "
        "be positive integers.");
    return 1;
  }

  char* ifname              = strdup(argv[1]);
  slave_id                  = atoi(argv[2]);
  int32_t loop_freq_hz      = atoi(argv[3]);
  initial_target_pos        = atoi(argv[4]);
  displ_from_initial_target = atoi(argv[5]);
  profile_velocity          = atoi(argv[6]);
  profile_accel             = atoi(argv[7]);
  profile_decel             = atoi(argv[8]);
  float  peak_current       = atof(argv[9]);
  float  continuous_current = atof(argv[10]);
  double max_motor_speed    = atof(argv[11]);

  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using loop frequency of %i hz", loop_freq_hz);
  MSG("Using target position 1 of %i counts", initial_target_pos);
  MSG("Using target position 2 of %i counts", displ_from_initial_target);
  MSG("Using profile velocity %u counts/s", profile_velocity);
  MSG("Using profile acceleration %u counts/s/s", profile_accel);
  MSG("Using profile deceleration %u counts/s/s", profile_decel);
  MSG("Using peak current of %f A", peak_current);
  MSG("Using continuous current of %f A", continuous_current);
  MSG("Using max_motor_speed of %lf cnts/sec", max_motor_speed);

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
  config.epd.over_speed_threshold = 0.0;  // Disable over speed protection.
  config.epd.low_position_limit   = 0.0;
  config.epd.high_position_limit =
      config.epd.low_position_limit;  // Disable position limits protection.
  config.epd.brake_engage_msec    = BRAKE_TIME_MSEC;
  config.epd.brake_disengage_msec = BRAKE_TIME_MSEC;
  config.epd.smooth_factor        = 0;

  MSG("Configuring %i as loop_period_ms", config.epd.loop_period_ms);

  jsd_set_slave_config(sds.jsd, slave_id, config);

  server_startup_s = jsd_time_get_mono_time_sec();

  sds_run(&sds, ifname, "/tmp/jsd_epd_prof_pos_test.csv");

  return 0;
}