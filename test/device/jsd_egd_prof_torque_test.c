#include <assert.h>
#include <float.h>
#include <string.h>

#include "jsd/jsd_egd_pub.h"
#include "jsd/jsd_time.h"
#include "jsd_test_utils.h"

#define NUM_FACTORS 5

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;
double       server_startup_s;
double       max_target_torque;
uint32_t     profile_accel;
uint32_t     profile_decel;
double       target_factors[NUM_FACTORS] = {0.0, 0.5, 0.8, -0.7, -1.0};

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
  const jsd_egd_state_t*  state = jsd_egd_get_state(sds->jsd, slave_id);

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
  fprintf(file, "%i, ", state->drive_temperature);

  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) { (void)self; };

void extract_data(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  jsd_egd_process(sds->jsd, slave_id);
}

void command(void* self) {
  static bool cmd_sent = 0;
  static double cmd_start_time = 0;

  single_device_server_t* sds = (single_device_server_t*)self;

  double now_s = jsd_time_get_mono_time_sec();

  jsd_egd_read(sds->jsd, slave_id);
  const jsd_egd_state_t* state = jsd_egd_get_state(sds->jsd, slave_id);

  // Wait 2 seconds after server starts to issue first reset.
  if ((now_s - server_startup_s) < 2.0) {
    return;
  }

  // Reset whenever not in OPERATION ENABLED state.
  if (!state->servo_enabled) {
    MSG("Sending reset.");
    jsd_egd_reset(sds->jsd, slave_id);
    return;
  }

  jsd_elmo_motion_command_prof_torque_t cmd;
  cmd.target_torque_amps = max_target_torque;

  // Change target torque every 10 seconds.
  if (!cmd_sent) { //iter % (sds->loop_rate_hz * 10) == 0) {

    cmd_sent = true;
    cmd_start_time = jsd_time_get_mono_time_sec();
  }

  if ((now_s - cmd_start_time) > 5.0) {
    cmd.target_torque_amps = 0.0;
  }

  jsd_egd_private_state_t* priv_state  = &sds->jsd->slave_states[slave_id].egd;
  MSG("Motor rated current %u", priv_state->motor_rated_current);
  MSG("RXPDO target torque %i", priv_state->rxpdo_prof.target_torque);
  jsd_egd_set_motion_command_prof_torque(sds->jsd, slave_id, cmd);
}

int main(int argc, char* argv[]) {
  if (argc != 7) {
    ERROR("Expecting exactly 6 arguments");
    MSG("Usage: jsd_egd_prof_torque_test <ifname> <egd_slave_index> "
        "<loop_freq_hz> <max_target_torque> <peak_current_amps> "
        "<continuous_current_amps>");
    MSG("Example: $ jsd_egd_prof_torque_test eth0 2 100 0.15 0.5 0.25");
    return 0;
  }

  char* ifname             = strdup(argv[1]);
  slave_id                 = atoi(argv[2]);
  int32_t loop_freq_hz     = atoi(argv[3]);
  max_target_torque        = atof(argv[4]);
  float peak_current       = atof(argv[5]);
  float continuous_current = atof(argv[6]);

  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using loop frequency of %i hz", loop_freq_hz);
  MSG("Using maximum target torque of %f A", max_target_torque);
  MSG("Using peak current of %f A", peak_current);
  MSG("Using continuous current of %f A", continuous_current);

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
  config.driver_type                 = JSD_DRIVER_TYPE_EGD;
  config.egd.drive_cmd_mode           = JSD_EGD_DRIVE_CMD_MODE_PROFILED;
  config.egd.max_motor_speed          = 3413333;
  config.egd.loop_period_ms           = 1000 / loop_freq_hz;
  config.egd.torque_slope             = 1e7;
  config.egd.max_profile_accel        = 1e6;
  config.egd.max_profile_decel        = 1e7;
  config.egd.velocity_tracking_error  = 1e8;
  config.egd.position_tracking_error  = 1e9;
  config.egd.peak_current_limit       = peak_current;
  config.egd.peak_current_time        = 3.0f;
  config.egd.continuous_current_limit = continuous_current;
  config.egd.motor_stuck_current_level_pct =
      0.0f;  // Disable motor stuck protection.
  config.egd.motor_stuck_velocity_threshold = 0.0f;
  config.egd.motor_stuck_timeout            = 0.0f;
  config.egd.over_speed_threshold = 0.0;  // Disable over speed protection.
  config.egd.low_position_limit   = 0.0;
  config.egd.high_position_limit =
      config.egd.low_position_limit;  // Disable position limits protection.
  config.egd.brake_engage_msec    = BRAKE_TIME_MSEC;
  config.egd.brake_disengage_msec = BRAKE_TIME_MSEC;
  config.egd.crc               = INT32_MIN;
  config.egd.drive_max_current_limit       = -FLT_MAX;
  config.egd.smooth_factor        = 0;
  config.egd.ctrl_gain_scheduling_mode =
      JSD_ELMO_GAIN_SCHEDULING_MODE_PRELOADED;

  MSG("Configuring %i as loop_period_ms", config.egd.loop_period_ms);

  jsd_set_slave_config(sds.jsd, slave_id, config);

  server_startup_s = jsd_time_get_mono_time_sec();

  sds_run(&sds, ifname, "/tmp/jsd_egd_prof_torque_test.csv");

  return 0;
}