#include <assert.h>
#include <float.h>
#include <string.h>

#include "jsd/jsd_egd_pub.h"
#include "jsd/jsd_sdo_pub.h"
#include "jsd_test_utils.h"

extern bool                       quit;
extern FILE*                      file;
uint8_t                           slave_id;
uint8_t                           first_command = 1;
double                            command_time;
jsd_egd_motion_command_prof_pos_t prof_pos_cmd;

void telemetry_header() {
  if (!file) {
    return;
  }

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
  fprintf(file, "async_sdo_in_prog, ");

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

  single_device_server_t* sds   = (single_device_server_t*)self;
  const jsd_egd_state_t*  state = jsd_egd_get_state(sds->jsd, slave_id);

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
  fprintf(file, "%u, ", state->async_sdo_in_prog);

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

void print_info(void* self) {
  assert(self);
  // single_device_server_t* sds = (single_device_server_t*)self;

  // const jsd_egd_state_t* state = jsd_egd_get_state(sds->jsd, slave_id);

  // MSG("cmd_pos: %i, actual_pos: %i", state->cmd_position,
  //    state->actual_position);
}

void extract_data(void* self) {
  single_device_server_t* sds = (single_device_server_t*)self;
  jsd_egd_read(sds->jsd, slave_id);
  jsd_egd_process(sds->jsd, slave_id);
}

void command(void* self) {
  single_device_server_t* sds   = (single_device_server_t*)self;
  const jsd_egd_state_t*  state = jsd_egd_get_state(sds->jsd, slave_id);

  if (first_command) {
    command_time  = jsd_timer_get_time_sec();
    first_command = 0;
  }

  if (state->actual_state_machine_state ==
      JSD_EGD_STATE_MACHINE_STATE_OPERATION_ENABLED) {
    if (jsd_timer_get_time_sec() - command_time > 2) {
      MSG("sending prof_pos command now");
      jsd_egd_set_motion_command_prof_pos(sds->jsd, slave_id, prof_pos_cmd);
      command_time = jsd_timer_get_time_sec();
    }
  } else {
    MSG("sending reset");
    jsd_egd_reset(sds->jsd, slave_id);
  }

  static uint64_t cmd_cnt = 1;
  if (cmd_cnt % 100 == 0) {
    float value = 1.0 + cmd_cnt / 100000.0;

    jsd_sdo_set_param_async(sds->jsd, slave_id, jsd_egd_tlc_to_do("PL"), 2,
                            JSD_SDO_DATA_FLOAT, (void*)&value);

    jsd_sdo_get_param_async(sds->jsd, slave_id, jsd_egd_tlc_to_do("PL"), 2,
                            JSD_SDO_DATA_FLOAT);
  }
  cmd_cnt++;
}

int main(int argc, char* argv[]) {
  if (argc != 7) {
    ERROR("Expecting exactly 6 arguments");
    MSG("%s%s", "Usage: jsd_egd_test <ifname> <egd_slave_index> <loop_freq_hz>",
        "<peak_current> <continuous_current> <max_motor_speed> ");
    MSG("Example: $ jsd_egd_test eth0 2 1000 0.75 0.25 10000");
    return 0;
  }

  char* ifname                = strdup(argv[1]);
  slave_id                    = atoi(argv[2]);
  uint32_t loop_freq_hz       = atoi(argv[3]);
  double   peak_current       = atof(argv[4]);
  double   continuous_current = atof(argv[5]);
  uint32_t max_motor_speed    = atoi(argv[6]);
  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using frequency of %i hz", loop_freq_hz);
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
  my_config.egd.drive_cmd_mode                = JSD_EGD_DRIVE_CMD_MODE_PROFILED;
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
  my_config.egd.crc                           = INT32_MIN;
  my_config.egd.drive_max_current_limit       = -FLT_MAX;

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  prof_pos_cmd.target_position  = max_motor_speed;
  prof_pos_cmd.profile_velocity = max_motor_speed;
  prof_pos_cmd.end_velocity     = 0;
  prof_pos_cmd.profile_accel    = max_motor_speed;
  prof_pos_cmd.profile_decel    = 2 * max_motor_speed;
  prof_pos_cmd.relative         = 1;

  sds_run(&sds, ifname, "/tmp/jsd_egd.csv");

  return 0;
}
