#include <assert.h>
#include <math.h>
#include <string.h>

#include "jsd/jsd_el2124_pub.h"
#include "jsd/jsd_el3602_pub.h"
#include "jsd/jsd_epd_pub.h"
#include "jsd/jsd_time.h"
#include "jsd_test_utils.h"

extern bool  quit;
extern FILE* file;
uint8_t      epd_slave_id;
uint8_t      el3602_slave_id;
uint8_t      el2124_slave_id;
double       server_startup_s;
double       amplitude;
double       sine_freq;
uint8_t      el2124_cmd_output[JSD_EL2124_NUM_CHANNELS] = {1, 0, 1, 0};

int16_t BRAKE_TIME_MSEC = 100;

void telemetry_header() {
  if (!file) {
    return;
  }

  fprintf(file, "rel_time_s, ");
  fprintf(file, "cycle_period_s, ");

  // EPD headers
  fprintf(file, "EPD_actual_position, ");
  fprintf(file, "EPD_actual_velocity, ");
  fprintf(file, "EPD_actual_current, ");
  fprintf(file, "EPD_cmd_position, ");
  fprintf(file, "EPD_cmd_velocity, ");
  fprintf(file, "EPD_cmd_current, ");
  fprintf(file, "EPD_cmd_max_current, ");
  fprintf(file, "EPD_cmd_ff_position, ");
  fprintf(file, "EPD_cmd_ff_velocity, ");
  fprintf(file, "EPD_cmd_ff_current, ");
  fprintf(file, "EPD_actual_state_machine_state, ");
  fprintf(file, "EPD_actual_mode_of_operation, ");
  fprintf(file, "EPD_sto_engaged, ");
  fprintf(file, "EPD_hall_state, ");
  fprintf(file, "EPD_motor_on, ");
  fprintf(file, "EPD_in_motion, ");
  fprintf(file, "EPD_servo_enabled, ");
  fprintf(file, "EPD_warning, ");
  fprintf(file, "EPD_target_reached, ");
  fprintf(file, "EPD_bus_voltage, ");
  fprintf(file, "EPD_analog_input_voltage, ");
  // Omitted digital inputs and digital output commands.
  fprintf(file, "EPD_drive_temperature, ");

  // EL3602 headers
  for (int i = 0; i < JSD_EL3602_NUM_CHANNELS; ++i) {
    fprintf(file, "EL3602_ch%d_raw_value, EL3602_ch%d_volts, ", i, i);
    fprintf(file, "EL3602_limit1_ch%d, EL3602_limit2_ch%d, ", i, i);
    fprintf(file, "EL3602_txPDO_state_ch%d, EL3602_txPDO_toggle_ch%d, ", i, i);
    fprintf(
        file,
        "EL3602_error_ch%d, EL3602_underrange_ch%d, EL3602_overrange_ch%d, ", i,
        i, i);
  }

  // EL2124 headers
  for (int i = 0; i < JSD_EL2124_NUM_CHANNELS; ++i) {
    fprintf(file, "EL2124_output_ch%d, ", i);
  }

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

  fprintf(file, "%lf, ", rel_time_s);
  fprintf(file, "%lf, ", cycle_period_s);

  single_device_server_t* sds = (single_device_server_t*)self;

  // EPD data
  const jsd_epd_state_t* epd_state = jsd_epd_get_state(sds->jsd, epd_slave_id);
  fprintf(file, "%i, ", epd_state->actual_position);
  fprintf(file, "%i, ", epd_state->actual_velocity);
  fprintf(file, "%lf, ", epd_state->actual_current);
  fprintf(file, "%i, ", epd_state->cmd_position);
  fprintf(file, "%i, ", epd_state->cmd_velocity);
  fprintf(file, "%lf, ", epd_state->cmd_current);
  fprintf(file, "%lf, ", epd_state->cmd_max_current);
  fprintf(file, "%i, ", epd_state->cmd_ff_position);
  fprintf(file, "%i, ", epd_state->cmd_ff_velocity);
  fprintf(file, "%lf, ", epd_state->cmd_ff_current);
  fprintf(file, "%u, ", epd_state->actual_state_machine_state);
  fprintf(file, "%u, ", epd_state->actual_mode_of_operation);
  fprintf(file, "%u, ", epd_state->sto_engaged);
  fprintf(file, "%u, ", epd_state->hall_state);
  fprintf(file, "%u, ", epd_state->motor_on);
  fprintf(file, "%u, ", epd_state->in_motion);
  fprintf(file, "%u, ", epd_state->servo_enabled);
  fprintf(file, "%u, ", epd_state->warning);
  fprintf(file, "%u, ", epd_state->target_reached);
  fprintf(file, "%lf, ", epd_state->bus_voltage);
  fprintf(file, "%lf, ", epd_state->analog_input_voltage);
  // Omitted digital inputs and digital output commands.
  fprintf(file, "%f, ", epd_state->drive_temperature);

  // EL3602 data
  const jsd_el3602_state_t* el3602_state =
      jsd_el3602_get_state(sds->jsd, el3602_slave_id);
  for (int i = 0; i < JSD_EL3602_NUM_CHANNELS; ++i) {
    fprintf(file, "%u, %lf, ", el3602_state->adc_value[i],
            el3602_state->voltage[i]);
    fprintf(file, "%u, %u, ", el3602_state->limit1[i], el3602_state->limit2[i]);
    fprintf(file, "%u, %u, ", el3602_state->txPDO_state[i],
            el3602_state->txPDO_toggle[i]);
    fprintf(file, "%u, %u, ", el3602_state->error[i],
            el3602_state->underrange[i]);
    fprintf(file, "%u, ", el3602_state->overrange[i]);
  }

  // EL2124 data
  const jsd_el2124_state_t* el2124_state =
      jsd_el2124_get_state(sds->jsd, el2124_slave_id);
  for (int i = 0; i < JSD_EL2124_NUM_CHANNELS; ++i) {
    fprintf(file, "%u, ", el2124_state->output[i]);
  }

  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);

  single_device_server_t* sds = (single_device_server_t*)self;

  const jsd_epd_state_t* epd_state = jsd_epd_get_state(sds->jsd, epd_slave_id);
  const jsd_el3602_state_t* el3602_state =
      jsd_el3602_get_state(sds->jsd, el3602_slave_id);
  const jsd_el2124_state_t* el2124_state =
      jsd_el2124_get_state(sds->jsd, el2124_slave_id);
  MSG("EPD_cmd_pos: %i, EPD_actual_pos: %i, EPD_actual_vel: %i, "
      "EPD_state_machine: %04x",
      epd_state->cmd_position, epd_state->actual_position,
      epd_state->actual_velocity, epd_state->actual_state_machine_state);
  MSG("EL3602_Ch0: %f V, EL3602_Ch1: %f V", el3602_state->voltage[0],
      el3602_state->voltage[1]);
  MSG("EL2124_Ch0: %u, EL2124_Ch1: %u, EL2124_Ch2: %u, EL2124_Ch3: %u",
      el2124_state->output[0], el2124_state->output[1], el2124_state->output[2],
      el2124_state->output[3]);
};

void extract_data(void* self) {
  assert(self);
  single_device_server_t* sds = (single_device_server_t*)self;

  jsd_epd_process(sds->jsd, epd_slave_id);
  jsd_el2124_process(sds->jsd, el2124_slave_id);
  jsd_el3602_read(sds->jsd, el3602_slave_id);
}

void command(void* self) {
  static int32_t iter             = 0;
  static int32_t pos_offset       = 0;
  static double  motion_startup_s = 0.0;
  static bool    first_motion_cmd = true;

  single_device_server_t* sds = (single_device_server_t*)self;

  double now_s = jsd_time_get_mono_time_sec();

  // Command EL2124
  // Toggle digital output every 10 seconds.
  if (iter % (sds->loop_rate_hz * 10) == 0) {
    el2124_cmd_output[0] ^= 0x01;
    el2124_cmd_output[1] ^= 0x01;
    el2124_cmd_output[2] ^= 0x01;
    el2124_cmd_output[3] ^= 0x01;
  }
  jsd_el2124_write_all_channels(sds->jsd, el2124_slave_id, el2124_cmd_output);
  ++iter;

  // Command EPD
  jsd_epd_read(sds->jsd, epd_slave_id);
  const jsd_epd_state_t* state = jsd_epd_get_state(sds->jsd, epd_slave_id);

  // Wait 2 seconds after server starts to issue first reset.
  if ((now_s - server_startup_s) < 2.0) {
    return;
  }

  // Reset whenever not in OPERATION ENABLED state.
  if (!state->servo_enabled) {
    MSG("Sending reset.");
    jsd_epd_reset(sds->jsd, epd_slave_id);
    return;
  }

  if (first_motion_cmd) {
    pos_offset       = state->actual_position;
    motion_startup_s = jsd_time_get_mono_time_sec();
    now_s = motion_startup_s;  // Force first value of time variable in
                               // sinusoidal command to be 0 for simplicity.
    first_motion_cmd = false;
  }

  jsd_elmo_motion_command_csp_t csp;
  double                        t = now_s - motion_startup_s;
  double                        w = 2.0 * M_PI * sine_freq;
  csp.target_position             = amplitude * sin(w * t) + pos_offset;
  csp.position_offset             = 0;
  csp.velocity_offset             = amplitude * w * cos(w * t);
  csp.torque_offset_amps          = 0.0;

  jsd_epd_set_motion_command_csp(sds->jsd, epd_slave_id, csp);
}

int main(int argc, char* argv[]) {
  if (argc != 11) {
    ERROR("Expecting exactly 10 arguments");
    MSG("Usage: jsd_epd_csp_sine_test <ifname> <epd_slave_index> "
        "<el3602_slave_index> <el2124_slave_index> <loop_freq_hz> "
        "<amplitude> <sine_freq> <peak_current_amps> <continuous_current_amps> "
        "<max_motor_speed> ");
    MSG("Example: $ jsd_epd_network_test eth0 4 2 3 100 25000 0.25 1.0 0.5 "
        "500000.0");
    return 0;
  }

  char* ifname              = strdup(argv[1]);
  epd_slave_id              = atoi(argv[2]);
  el3602_slave_id           = atoi(argv[3]);
  el2124_slave_id           = atoi(argv[4]);
  int32_t loop_freq_hz      = atoi(argv[5]);
  amplitude                 = atof(argv[6]);
  sine_freq                 = atof(argv[7]);
  float  peak_current       = atof(argv[8]);
  float  continuous_current = atof(argv[9]);
  double max_motor_speed    = atof(argv[10]);

  MSG("Configuring device %s", ifname);
  MSG("Using EPD at %d, EL3602 at %d, and EL2124 at %d", epd_slave_id,
      el3602_slave_id, el2124_slave_id);
  MSG("Using loop frequency of %i hz", loop_freq_hz);
  MSG("Using amplitude of %lf counts", amplitude);
  MSG("Using sine frequency of %lf hz", sine_freq);
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

  // Configure EPD

  jsd_slave_config_t epd_config = {0};

  snprintf(epd_config.name, JSD_NAME_LEN, "kukulkan");
  epd_config.configuration_active         = true;
  epd_config.product_code                 = JSD_EPD_PRODUCT_CODE;
  epd_config.epd.max_motor_speed          = max_motor_speed;
  epd_config.epd.loop_period_ms           = 1000 / loop_freq_hz;
  epd_config.epd.torque_slope             = 1e7;
  epd_config.epd.max_profile_accel        = 1e6;
  epd_config.epd.max_profile_decel        = 1e7;
  epd_config.epd.quick_stop_decel         = 1e7;
  epd_config.epd.stop_decel               = 1e7;
  epd_config.epd.velocity_tracking_error  = 1e8;
  epd_config.epd.position_tracking_error  = 1e9;
  epd_config.epd.peak_current_limit       = peak_current;
  epd_config.epd.peak_current_time        = 3.0f;
  epd_config.epd.continuous_current_limit = continuous_current;
  epd_config.epd.motor_stuck_current_level_pct =
      0.0f;  // Disable motor stuck protection.
  epd_config.epd.motor_stuck_velocity_threshold = 0.0f;
  epd_config.epd.motor_stuck_timeout            = 0.0f;
  epd_config.epd.over_speed_threshold = 0.0;  // Disable over speed protection.
  epd_config.epd.low_position_limit   = 0.0;
  epd_config.epd.high_position_limit =
      epd_config.epd.low_position_limit;  // Disable position limits protection.
  epd_config.epd.brake_engage_msec    = BRAKE_TIME_MSEC;
  epd_config.epd.brake_disengage_msec = BRAKE_TIME_MSEC;
  epd_config.epd.smooth_factor        = 0;

  jsd_set_slave_config(sds.jsd, epd_slave_id, epd_config);

  // Configure EL3602

  jsd_slave_config_t el3602_config = {0};

  snprintf(el3602_config.name, JSD_NAME_LEN, "unicorn");
  el3602_config.configuration_active = true;
  el3602_config.product_code         = JSD_EL3602_PRODUCT_CODE;
  el3602_config.el3602.range[0]      = JSD_EL3602_RANGE_10V;
  el3602_config.el3602.range[1]      = JSD_EL3602_RANGE_10V;
  el3602_config.el3602.filter[0]     = JSD_BECKHOFF_FILTER_30000HZ;
  el3602_config.el3602.filter[1]     = JSD_BECKHOFF_FILTER_30000HZ;

  jsd_set_slave_config(sds.jsd, el3602_slave_id, el3602_config);

  // Configure EL2124

  jsd_slave_config_t el2124_config = {0};

  snprintf(el2124_config.name, JSD_NAME_LEN, "bigfoot");
  el2124_config.configuration_active = true;
  el2124_config.product_code         = JSD_EL2124_PRODUCT_CODE;

  jsd_set_slave_config(sds.jsd, el2124_slave_id, el2124_config);

  server_startup_s = jsd_time_get_mono_time_sec();

  sds_run(&sds, ifname, "/tmp/jsd_epd_network_test.csv");

  return 0;
}
