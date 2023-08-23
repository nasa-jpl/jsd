#include <assert.h>
#include <string.h>

// device includes as necessary by application
#include "jsd/jsd_el6001_pub.h"
#include "jsd/jsd_el6001_types.h"
#include "jsd_test_utils.h"

#define HEX_FORMAT "0x%02X"
#define LOOP_FREQUENCY_HZ (500) // Rate to update ethercat devices
#define USER_LOOP_FREQUENCY_HZ (1) // Rate to send and receive user commands in test
#define NSEC_PER_SEC (1000000000L)
#define LOOP_PERIOD_NS (NSEC_PER_SEC / LOOP_FREQUENCY_HZ) // ns
#define USER_LOOP_RATE_SEC (1.0 / USER_LOOP_FREQUENCY_HZ) // sec
#define PRINT_RATE_HZ (1) // Hz
#define NUM_RESOLVERS (1)
#define MAX_NUM_RESOLVERS (6)
#define ID 0
#define OP_CODE_SEND_DATA (1)
#define NUM_BYTES_BEFORE_RESOLVERS (2)
#define NUM_BYTES_PER_RESOLVER (3)
#define NUM_RESOLVER_BYTES (NUM_BYTES_PER_RESOLVER * MAX_NUM_RESOLVERS)
#define TOTAL_NUM_BYTES (NUM_BYTES_BEFORE_RESOLVERS + NUM_RESOLVER_BYTES + 1/*checksum*/)

extern bool  quit;
extern FILE* file;
uint8_t      slave_id;

void telemetry_header() {  
  if (!file) {
    return;
  }
  fprintf(file, "EL6001_controlword_last, EL6001_controlword, ");
  fprintf(file, "EL6001_data_out_0, EL6001_data_out_1, ");
  fprintf(file, "EL6001_data_out_2, EL6001_data_out_3, ");    
  fprintf(file, "EL6001_data_out_4, EL6001_data_out_5, ");    
  fprintf(file, "EL6001_data_out_6, EL6001_data_out_7, ");    
  fprintf(file, "EL6001_data_out_8, EL6001_data_out_9, ");    
  fprintf(file, "EL6001_data_out_10, EL6001_data_out_11, ");    
  fprintf(file, "EL6001_data_out_12, EL6001_data_out_13, ");    
  fprintf(file, "EL6001_data_out_14, EL6001_data_out_15, ");    
  fprintf(file, "EL6001_data_out_16, EL6001_data_out_17, ");    
  fprintf(file, "EL6001_data_out_18, EL6001_data_out_19, ");    
  fprintf(file, "EL6001_data_out_20, EL6001_data_out_21, ");    

  fprintf(file, "EL6001_statusword_last, EL6001_statusword, ");
  fprintf(file, "EL6001_data_in_0, EL6001_data_in_1, ");
  fprintf(file, "EL6001_data_in_2, EL6001_data_in_3, ");    
  fprintf(file, "EL6001_data_in_4, EL6001_data_in_5, ");    
  fprintf(file, "EL6001_data_in_6, EL6001_data_in_7, ");    
  fprintf(file, "EL6001_data_in_8, EL6001_data_in_9, ");    
  fprintf(file, "EL6001_data_in_10, EL6001_data_in_11, ");    
  fprintf(file, "EL6001_data_in_12, EL6001_data_in_13, ");    
  fprintf(file, "EL6001_data_in_14, EL6001_data_in_15, ");    
  fprintf(file, "EL6001_data_in_16, EL6001_data_in_17, ");    
  fprintf(file, "EL6001_data_in_18, EL6001_data_in_19, ");    
  fprintf(file, "EL6001_data_in_20, EL6001_data_in_21, ");       
  
  fprintf(file, "\n");
}

void telemetry_data(void* self) {
  assert(self);

  if (!file) {
    return;
  }

  single_device_server_t*   sds   = (single_device_server_t*)self;
  const jsd_el6001_private_state_t* state = jsd_el6001_get_state(sds->jsd, slave_id);

  fprintf(file, "%u, %u,", state->controlword_user_last, state->controlword_user);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[0], state->pub.transmit_bytes[1]);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[2], state->pub.transmit_bytes[3]);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[4], state->pub.transmit_bytes[5]);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[6], state->pub.transmit_bytes[7]);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[8], state->pub.transmit_bytes[9]);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[10], state->pub.transmit_bytes[11]);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[12], state->pub.transmit_bytes[13]);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[14], state->pub.transmit_bytes[15]);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[16], state->pub.transmit_bytes[17]);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[18], state->pub.transmit_bytes[19]);
  fprintf(file, "%u, %u,", state->pub.transmit_bytes[20], state->pub.transmit_bytes[21]);

  fprintf(file, "%u, %u,", state->statusword_last, state->statusword);
  fprintf(file, "%u, %u,", state->pub.received_bytes[0], state->pub.received_bytes[1]);
  fprintf(file, "%u, %u,", state->pub.received_bytes[2], state->pub.received_bytes[3]);
  fprintf(file, "%u, %u,", state->pub.received_bytes[4], state->pub.received_bytes[5]);
  fprintf(file, "%u, %u,", state->pub.received_bytes[6], state->pub.received_bytes[7]);
  fprintf(file, "%u, %u,", state->pub.received_bytes[8], state->pub.received_bytes[9]);
  fprintf(file, "%u, %u,", state->pub.received_bytes[10], state->pub.received_bytes[11]);
  fprintf(file, "%u, %u,", state->pub.received_bytes[12], state->pub.received_bytes[13]);
  fprintf(file, "%u, %u,", state->pub.received_bytes[14], state->pub.received_bytes[15]);
  fprintf(file, "%u, %u,", state->pub.received_bytes[16], state->pub.received_bytes[17]);
  fprintf(file, "%u, %u,", state->pub.received_bytes[18], state->pub.received_bytes[19]);
  fprintf(file, "%u, %u,", state->pub.received_bytes[20], state->pub.received_bytes[21]);
  
  fprintf(file, "\n");
  fflush(file);
}

void print_info(void* self) {
  assert(self);

  single_device_server_t*   sds   = (single_device_server_t*)self;
  const jsd_el6001_private_state_t* state = jsd_el6001_get_state(sds->jsd, slave_id);
  MSG("Controlword_user: %u ", state->controlword_user);
  MSG("Statusword: %u ", state->statusword);
}

void extract_data(void* self) {
  single_device_server_t* sds = (single_device_server_t*)self;
  jsd_el6001_read(sds->jsd, slave_id);
}

void command(void* self) { (void)self; };


int main(int argc, char* argv[]) {
  if (argc != 4) {
    ERROR("Expecting exactly 3 arguments");
    MSG("Usage: jsd_el6001_test <ifname> <el6001_slave_index> <loop_freq_hz>");
    MSG("Example: $ jsd_el6001_test eth0 2 1000");
    return 0;
  }

  char* ifname          = strdup(argv[1]);
  slave_id              = atoi(argv[2]);
  uint32_t loop_freq_hz = atoi(argv[3]);

  MSG("Configuring device %s, using slave %d", ifname, slave_id);
  MSG("Using frequency of %i hz", loop_freq_hz);

  single_device_server_t sds;

  sds_set_telemetry_header_callback(&sds, telemetry_header);
  sds_set_telemetry_data_callback(&sds, telemetry_data);
  sds_set_print_info_callback(&sds, print_info);
  sds_set_extract_data_callback(&sds, extract_data);
  sds_set_command_callback(&sds, command);

  sds_setup(&sds, loop_freq_hz);

  // set device configuration here
  jsd_slave_config_t my_config = {0};
  
  my_config.el6001.baud_rate = JSD_EL6001_BAUD_RATE_115200;
  
  snprintf(my_config.name, JSD_NAME_LEN, "my_el6001_device");
  my_config.configuration_active = true;
  my_config.product_code         = JSD_EL6001_PRODUCT_CODE;

  jsd_set_slave_config(sds.jsd, slave_id, my_config);

  if (!jsd_init(sds.jsd, ifname, 1)) {
    ERROR("Could not init jsd");
    return 0;
  }

  jsd_read(sds.jsd, EC_TIMEOUTRET);

  int8_t counter = 2;
  // bool quit = false;
  const jsd_el6001_private_state_t* state = jsd_el6001_get_state(sds.jsd, slave_id);

  // initiate first transmit and for next time all data is received to prevent idle loop
  // transmit data does not get deleted automatically
  MSG("Transmitting opcode %d and counter %d", OP_CODE_SEND_DATA, counter);
  jsd_el6001_set_transmit_data_8bits(sds.jsd, slave_id, 0/*byte_number*/, 1);
  jsd_el6001_set_transmit_data_8bits(sds.jsd, slave_id, 1/*byte_number*/, 0);
  jsd_el6001_set_transmit_data_8bits(sds.jsd, slave_id, 2/*byte_number*/, 2);
  jsd_el6001_request_transmit_data(sds.jsd, slave_id, 3/*num_bytes*/);

  while(!quit){
    jsd_read(sds.jsd, EC_TIMEOUTRET);

    // read PDOs, updates statusword
    jsd_el6001_read(sds.jsd, slave_id);  

    MSG("Controlword_user: %u ", state->controlword_user);
    MSG("Statusword: %u ", state->statusword);

    jsd_el6001_process(sds.jsd, slave_id);

    jsd_write(sds.jsd);

    // sleep(1.0);
    jsd_timer_process(sds.jsd_timer);
  }

  return 0;
}
