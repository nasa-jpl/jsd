#include <assert.h>
#include <math.h>
#include <string.h>

#include "jsd/jsd_epd_pub.h"
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
int32_t      loop_freq_hz;

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
  static int32_t pos_offset                  = 0;
  static double  motion_startup_s            = 0.0;
  static bool    first_motion_cmd            = true;
  static bool    motion_config_sent          = false;
  static size_t  motion_config_req_processed = 0;

  single_device_server_t* sds = (single_device_server_t*)self;

  double now_s = jsd_time_get_mono_time_sec();

  jsd_epd_read(sds->jsd, slave_id);
  const jsd_epd_state_t* state = jsd_epd_get_state(sds->jsd, slave_id);

  if (!motion_config_sent) {
    // Set interpolation time period.
    uint8_t loop_period_ms = 1000 / loop_freq_hz;
    if (!jsd_sdo_set_param_async(sds->jsd, slave_id, 0x60C2, 1, JSD_SDO_DATA_U8,
                                 &loop_period_ms, 0)) {
      ERROR("Could not send asynchronous request to set loop period.");
      assert(false);
    }
    // Set velocity tracking error.
    double velocity_tracking_error = 1e8;
    if (!jsd_sdo_set_param_async(sds->jsd, slave_id, jsd_epd_lc_to_do("ER"), 2,
                                 JSD_SDO_DATA_DOUBLE, &velocity_tracking_error,
                                 1)) {
      ERROR(
          "Could not send asynchronous request to set velocity tracking "
          "error.");
      assert(false);
    }
    // Set position tracking error.
    double position_tracking_error = 1e9;
    if (!jsd_sdo_set_param_async(sds->jsd, slave_id, jsd_epd_lc_to_do("ER"), 3,
                                 JSD_SDO_DATA_DOUBLE, &position_tracking_error,
                                 2)) {
      ERROR(
          "Could not send asynchronous request to set position tracking "
          "error.");
      assert(false);
    }
    // Disable position limits protection.
    double low_position_limit = 0.0;
    if (!jsd_sdo_set_param_async(sds->jsd, slave_id, jsd_epd_lc_to_do("LL"), 3,
                                 JSD_SDO_DATA_DOUBLE, &low_position_limit, 3)) {
      ERROR("Could not send asynchronous request to set low position limit.");
      assert(false);
    }
    double high_position_limit = 0.0;
    if (!jsd_sdo_set_param_async(sds->jsd, slave_id, jsd_epd_lc_to_do("HL"), 3,
                                 JSD_SDO_DATA_DOUBLE, &high_position_limit,
                                 4)) {
      ERROR("Could not send asynchronous request to set high position limit.");
      assert(false);
    }

    motion_config_sent = true;
    return;
  }

  if (motion_config_req_processed != 5) {
    jsd_sdo_req_t response;
    while (jsd_sdo_pop_response_queue(sds->jsd, &response)) {
      switch (response.app_id) {
        case 0:
          if (!response.success) {
            ERROR("Could not set loop period through asynchronous SDO.");
            assert(false);
          } else {
            ++motion_config_req_processed;
          }
          break;
        case 1:
          if (!response.success) {
            ERROR("Could not set velocity tracking through asynchronous SDO.");
            assert(false);
          } else {
            ++motion_config_req_processed;
          }
          break;
        case 2:
          if (!response.success) {
            ERROR(
                "Could not set position tracking error through asynchronous "
                "SDO.");
            assert(false);
          } else {
            ++motion_config_req_processed;
          }
          break;
        case 3:
          if (!response.success) {
            ERROR("Could not set low position limit through asynchronous SDO.");
            assert(false);
          } else {
            ++motion_config_req_processed;
          }
          break;
        case 4:
          if (!response.success) {
            ERROR(
                "Could not set high position limit through asynchronous SDO.");
            assert(false);
          } else {
            ++motion_config_req_processed;
          }
          break;
        default:
          MSG_DEBUG(
              "Received an SDO response that does not correspond to the motion "
              "configuration parameters.");
      }
    }
    return;
  }

  // Wait 2 seconds after server starts to issue first reset.
  if ((now_s - server_startup_s) < 2.0) {
    return;
  }

  // Reset whenever not in OPERATION ENABLED state.
  if (!state->servo_enabled) {
    MSG("Sending reset.");
    jsd_epd_reset(sds->jsd, slave_id);
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

  jsd_epd_set_sil_input_r2(sds->jsd, slave_id, 1, target_position);
}

int main(int argc, char* argv[]) {
  if (argc != 8) {
    ERROR("Expecting exactly 7 arguments");
    MSG("Usage: jsd_epd_csp_sine_sil_test <ifname> <epd_slave_index> "
        "<loop_freq_hz> <amplitude> <sine_freq> "
        "<peak_current_amps> <continuous_current_amps>");
    MSG("Example: $ jsd_epd_csp_sine_sil_test eth0 2 100 25000 0.25 0.25 0.45");
    return 0;
  }

  char* ifname             = strdup(argv[1]);
  slave_id                 = atoi(argv[2]);
  loop_freq_hz             = atoi(argv[3]);
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
  config.configuration_active         = true;
  config.product_code                 = JSD_EPD_PRODUCT_CODE_STD_FW;
  config.epd.peak_current_limit       = peak_current;
  config.epd.peak_current_time        = 3.0f;
  config.epd.continuous_current_limit = continuous_current;
  config.epd.motor_stuck_current_level_pct =
      0.0f;  // Disable motor stuck protection.
  config.epd.motor_stuck_velocity_threshold = 0.0f;
  config.epd.motor_stuck_timeout            = 0.0f;
  config.epd.over_speed_threshold = 0.0;  // Disable over speed protection.
  config.epd.brake_engage_msec    = BRAKE_TIME_MSEC;
  config.epd.brake_disengage_msec = BRAKE_TIME_MSEC;
  config.epd.use_sil              = true;
  config.epd.sil.inputs_r2_num    = 1;

  jsd_set_slave_config(sds.jsd, slave_id, config);

  server_startup_s = jsd_time_get_mono_time_sec();

  sds_run(&sds, ifname, "/tmp/jsd_epd_csp_sine_sil_test.csv");

  return 0;
}