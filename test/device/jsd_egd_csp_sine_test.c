#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "jsd/jsd_egd_pub.h"
#include "jsd_test_utils.h"

int32_t BRAKE_TIME_MSEC = 100;

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;
double       amplitude;
double       sine_freq;
double       server_startup_sec;
double       last_rel_time_sec = 0;
double       startup_sec;
uint8_t      enable_velocity_offset;
uint8_t      first_time = 1;
int32_t      loop_freq_hz;
int32_t      pos_offset;

void telemetry_header() {
  if (!file) {
    return;
  }
  fprintf(file, "rel_time_sec, ");
  fprintf(file, "jitter_ms, ");

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
  fprintf(file, "in_motion, ");
  fprintf(file, "warning, ");
  fprintf(file, "target_reached, ");
  fprintf(file, "motor_on, ");

  fprintf(file, "bus_voltage, ");
  fprintf(file, "analog_input_voltage, ");
  // omitting DINs
  // omitting DOUTs
  fprintf(file, "drive_temperature, ");

  fprintf(file, "\n");
}
void telemetry_data(void* self) {
  assert(self);
  if (!file) {
    return;
  }

  double now_sec      = jsd_timer_get_time_sec();
  double rel_time_sec = now_sec - server_startup_sec;
  double jitter       = rel_time_sec - last_rel_time_sec;
  last_rel_time_sec   = rel_time_sec;

  single_device_server_t* sds   = (single_device_server_t*)self;
  const jsd_egd_state_t*  state = jsd_egd_get_state(sds->jsd, slave_id);

  fprintf(file, "%lf, ", rel_time_sec);
  fprintf(file, "%lf, ", jitter * 1000.0);

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
  fprintf(file, "%u, ", state->in_motion);
  fprintf(file, "%u, ", state->warning);
  fprintf(file, "%u, ", state->target_reached);
  fprintf(file, "%u, ", state->motor_on);

  fprintf(file, "%lf, ", state->bus_voltage);
  fprintf(file, "%lf, ", state->analog_input_voltage);
  // omitting DINs
  // omitting DOUTs
  fprintf(file, "%u, ", state->drive_temperature);

  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) { (void)self; };

void extract_data(void* self) {
  single_device_server_t* sds = (single_device_server_t*)self;
  jsd_egd_process(sds->jsd, slave_id);
}

void command(void* self) {
  single_device_server_t* sds   = (single_device_server_t*)self;
  const jsd_egd_state_t*  state = jsd_egd_get_state(sds->jsd, slave_id);

  jsd_egd_read(sds->jsd, slave_id);
  jsd_egd_process(sds->jsd, slave_id);

  jsd_egd_motion_command_csp_t csp;
  csp.position_offset    = 0;
  csp.torque_offset_amps = 0;

  double now_sec = jsd_timer_get_time_sec();

  // wait 2 seconds after server starts to issue first reset and command
  if ((now_sec - server_startup_sec) < 2.0) {
    return;
  }

  // reset if faulted
  if (!state->motor_on) {
    MSG("sending reset");
    jsd_egd_reset(sds->jsd, slave_id);
    first_time = 1;
    return;
  }

  // send cmd = actual to initiate trigger to CSP mode
  if (first_time &&
      state->actual_mode_of_operation != JSD_EGD_MODE_OF_OPERATION_CSP) {
    // MSG("Sending first csp command now");
    csp.target_position = state->actual_position;
    csp.velocity_offset = 0;
    jsd_egd_set_motion_command_csp(sds->jsd, slave_id, csp);
  } else {
    if (first_time) {
      first_time  = 0;
      startup_sec = jsd_timer_get_time_sec();
      pos_offset  = state->actual_position;
    }

    // MSG("act pos: %i  first_cmd: %i", state->actual_position,
    //    (int32_t)(amplitude * sin(M_PI / 2.0)));
    // MSG("target_pos: %i dt: %lf", csp.target_position, dt);

    double dt           = now_sec - startup_sec;
    double U            = 2.0 * M_PI * sine_freq;
    csp.target_position = amplitude * sin(U * dt) + pos_offset;
    if (enable_velocity_offset) {
      csp.velocity_offset = amplitude * U * cos(U * dt);
    } else {
      csp.velocity_offset = 0;
    }

    jsd_egd_set_motion_command_csp(sds->jsd, slave_id, csp);
  }
}

int main(int argc, char* argv[]) {
  if (argc != 10) {
    ERROR("Expecting exactly 9 arguments");
    MSG("%s%s%s", "Usage: jsd_egd_csp_sine_test <ifname> <egd_slave_index> ",
        "<loop_freq_hz> <amplitude> <sine_freq> <enable_velocity_offset> ",
        "<peak_current_amps> <continuous_current_amps> <max_motor_speed> ");
    MSG("Example: $ jsd_egd_sine_test eth0 2 100 25000 0.25 1 0.25 0.45 50000");
    return 0;
  }

  char* ifname                = strdup(argv[1]);
  slave_id                    = atoi(argv[2]);
  loop_freq_hz                = atoi(argv[3]);
  amplitude                   = atof(argv[4]);
  sine_freq                   = atof(argv[5]);
  enable_velocity_offset      = atoi(argv[6]);
  double   peak_current       = atof(argv[7]);
  double   continuous_current = atof(argv[8]);
  uint32_t max_motor_speed    = atoi(argv[9]);

  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using frequency of %i hz", loop_freq_hz);
  MSG("Using amplitude of %lf counts", amplitude);
  MSG("Using sine_freq of %lf hz", sine_freq);
  MSG("Using peak_current of %lf mA", peak_current);
  MSG("Using continuous current of %lf mA", continuous_current);
  MSG("Using max_motor_speed of %i cnts/sec", max_motor_speed);

  single_device_server_t sds;

  sds_set_telemetry_header_callback(&sds, telemetry_header);
  sds_set_telemetry_data_callback(&sds, telemetry_data);
  sds_set_print_info_callback(&sds, print_info);
  sds_set_extract_data_callback(&sds, extract_data);
  sds_set_command_callback(&sds, command);

  sds_setup(&sds, loop_freq_hz);

  jsd_slave_config_t my_config = {0};

  snprintf(my_config.name, JSD_NAME_LEN, "la chupacabra");
  my_config.configuration_active              = true;
  my_config.product_code                      = JSD_EGD_PRODUCT_CODE;
  my_config.egd.drive_cmd_mode                = JSD_EGD_DRIVE_CMD_MODE_CS;
  my_config.egd.max_motor_speed               = max_motor_speed;
  my_config.egd.loop_period_ms                = 1000 / loop_freq_hz;
  my_config.egd.torque_slope                  = 1e7;
  my_config.egd.max_profile_accel             = 1e6;
  my_config.egd.max_profile_decel             = 1e7;
  my_config.egd.velocity_tracking_error       = 1e8;
  my_config.egd.position_tracking_error       = 1e9;
  my_config.egd.peak_current_limit            = peak_current;
  my_config.egd.peak_current_time             = 3.0;
  my_config.egd.continuous_current_limit      = continuous_current;
  my_config.egd.motor_stuck_current_level_pct = 0;
  my_config.egd.motor_stuck_timeout           = 3.0;
  my_config.egd.over_speed_threshold          = 0;
  my_config.egd.low_position_limit            = 0;
  my_config.egd.high_position_limit           = 0;
  my_config.egd.brake_engage_msec             = BRAKE_TIME_MSEC;
  my_config.egd.brake_disengage_msec          = BRAKE_TIME_MSEC;
  my_config.egd.crc                           = INT32_MIN;
  my_config.egd.drive_max_current_limit       = -FLT_MAX;

  MSG("Configuring %i as loop_period_ms", my_config.egd.loop_period_ms);

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  server_startup_sec = jsd_timer_get_time_sec();

  sds_run(&sds, ifname, "/tmp/jsd_egd_csp_sine_test.csv");

  return 0;
}
