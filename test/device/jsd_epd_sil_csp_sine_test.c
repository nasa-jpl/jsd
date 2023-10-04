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

int16_t BRAKE_TIME_MSEC = 100;

typedef struct {
  int32_t loop_freq_hz;
  double  velocity_tracking_error;
  double  position_tracking_error;
  double  low_position_limit;
  double  high_position_limit;
} jsd_sil_csp_sine_test_config_t;

jsd_sil_csp_sine_test_config_t custom_config_test_data;

int custom_config_test_callback(ecx_contextt* ecx_context, uint16_t slave_id,
                                void* test_data_ptr) {
  jsd_sil_csp_sine_test_config_t* test_data =
      (jsd_sil_csp_sine_test_config_t*)test_data_ptr;

  // Set interpolation time period.
  uint8_t loop_period_ms = 1000 / test_data->loop_freq_hz;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, 0x60C2, 1,
                                  JSD_SDO_DATA_U8, &loop_period_ms)) {
    ERROR("Could not set loop period.");
    return 0;
  }
  // Set tracking error parameters.
  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("ER"), 2,
          JSD_SDO_DATA_DOUBLE, &test_data->velocity_tracking_error)) {
    ERROR("Could not set velocity tracking error.");
    return 0;
  }
  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("ER"), 3,
          JSD_SDO_DATA_DOUBLE, &test_data->position_tracking_error)) {
    ERROR("Could not set position tracking error.");
    return 0;
  }
  // Set position limits protection parameters.
  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("LL"), 3,
          JSD_SDO_DATA_DOUBLE, &test_data->low_position_limit)) {
    ERROR("Could not set low position limit.");
    return 0;
  }
  if (!jsd_sdo_set_param_blocking(
          ecx_context, slave_id, jsd_epd_sil_lc_to_do("HL"), 3,
          JSD_SDO_DATA_DOUBLE, &test_data->high_position_limit)) {
    ERROR("Could not set high position limit.");
    return 0;
  }

  return 1;
}

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
  fprintf(file, "%lf, ", jsd_epd_sil_get_sil_r2_input(sds->jsd, slave_id, 1));
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
  static int32_t pos_offset                  = 0;
  static double  motion_startup_s            = 0.0;
  static bool    first_motion_cmd            = true;

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
    return;
  }

  if (first_motion_cmd) {
    pos_offset       = state->actual_position;
    motion_startup_s = jsd_time_get_mono_time_sec();
    now_s = motion_startup_s;  // Force first value of time variable in
                               // sinusoidal command to be 0 for simplicity.
    first_motion_cmd = false;
  }

  double w               = 2.0 * M_PI * sine_freq;
  double t               = now_s - motion_startup_s;
  double target_position = amplitude * sin(w * t) + pos_offset;

  jsd_epd_sil_set_sil_r2_input(sds->jsd, slave_id, 1, target_position);
}

int main(int argc, char* argv[]) {
  if (argc != 8) {
    ERROR("Expecting exactly 7 arguments");
    MSG("Usage: jsd_epd_sil_csp_sine_test <ifname> <epd_slave_index> "
        "<loop_freq_hz> <amplitude> <sine_freq> "
        "<peak_current_amps> <continuous_current_amps> ");
    MSG("Example: $ jsd_epd_csp_sine_sil_test eth0 2 100 25000 0.25 0.25 0.45");
    return 0;
  }

  char* ifname             = strdup(argv[1]);
  slave_id                 = atoi(argv[2]);
  int32_t loop_freq_hz     = atoi(argv[3]);
  amplitude                = atof(argv[4]);
  sine_freq                = atof(argv[5]);
  float peak_current       = atof(argv[6]);
  float continuous_current = atof(argv[7]);

  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using loop frequency of %i hz", loop_freq_hz);
  MSG("Using amplitude of %lf counts", amplitude);
  MSG("Using sine frequency of %lf hz", sine_freq);
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
  config.configuration_active             = true;
  config.driver_type                      = JSD_DRIVER_TYPE_EPD_SIL;
  config.epd_sil.peak_current_limit       = peak_current;
  config.epd_sil.peak_current_time        = 3.0f;
  config.epd_sil.continuous_current_limit = continuous_current;
  config.epd_sil.motor_stuck_current_level_pct =
      0.0f;  // Disable motor stuck protection.
  config.epd_sil.motor_stuck_velocity_threshold = 0.0f;
  config.epd_sil.motor_stuck_timeout            = 0.0f;
  config.epd_sil.over_speed_threshold = 0.0;  // Disable over speed protection.
  config.epd_sil.brake_engage_msec    = BRAKE_TIME_MSEC;
  config.epd_sil.brake_disengage_msec = BRAKE_TIME_MSEC;
  config.epd_sil.sil_r2_inputs_num    = 1;

  custom_config_test_data.loop_freq_hz            = loop_freq_hz;
  custom_config_test_data.velocity_tracking_error = 1e8;
  custom_config_test_data.position_tracking_error = 1e9;
  // Disable position limits protection.
  custom_config_test_data.low_position_limit  = 0.0;
  custom_config_test_data.high_position_limit = 0.0;
  config.epd_sil.PO2SO_config_user_data       = &custom_config_test_data;
  config.epd_sil.PO2SO_config_user_callback   = &custom_config_test_callback;

  jsd_set_slave_config(sds.jsd, slave_id, config);

  server_startup_s = jsd_time_get_mono_time_sec();

  sds_run(&sds, ifname, "/tmp/jsd_epd_sil_csp_sine_test.csv");

  return 0;
}