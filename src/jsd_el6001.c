#include "jsd/jsd_el6001.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#include "jsd/jsd.h"
#include "jsd/jsd_sdo.h"

#define HEX_FORMAT "0x%02X"

const char* jsd_el6001_baud_rate_strings[] = {
    [JSD_EL6001_BAUD_RATE_2400]     = "2400 bps",
    [JSD_EL6001_BAUD_RATE_4800]     = "4800 bps",
    [JSD_EL6001_BAUD_RATE_9600]     = "9600 bps",
    [JSD_EL6001_BAUD_RATE_12000]    = "12000 bps",
    [JSD_EL6001_BAUD_RATE_14400]    = "14400 bps",
    [JSD_EL6001_BAUD_RATE_19200]    = "19200 bps",
    [JSD_EL6001_BAUD_RATE_38400]    = "38400 bps",
    [JSD_EL6001_BAUD_RATE_57600]    = "57600 bps",
    [JSD_EL6001_BAUD_RATE_115200]   = "115200 bps",    
};

// Print functions
static void jsd_el6001_print_controlword(const char* prefix, uint16_t controlword, uint16_t slave_id) {
  uint8_t output_length = controlword >> JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_0;

  MSG(
    "EL6001 ID %d: %s("HEX_FORMAT"), %s%s%s%s%d output bytes",
    slave_id,
    prefix,
    controlword,
    (controlword & (1 << JSD_EL6001_CONTROLWORD_TRANSMIT_REQUEST)) ? "Transmit Request | " : "",
    (controlword & (1 << JSD_EL6001_CONTROLWORD_RECEIVE_ACCEPTED)) ? "Receive Accepted | " : "",
    (controlword & (1 << JSD_EL6001_CONTROLWORD_INIT_REQUEST))     ? "Init Request | "     : "",
    (controlword & (1 << JSD_EL6001_CONTROLWORD_SEND_CONTINUOUS))  ? "Send Continuous | "  : "",
    output_length);
}

static char* jsd_el6001_sms_to_string(jsd_el6001_sms_t sms) {
  switch (sms)
  {
    case JSD_EL6001_SMS_INITING:                                return "INITING_SERIAL_COMMUNICATION";
    case JSD_EL6001_SMS_WAITING_FOR_TERMINAL_TO_INIT:           return "WAITING_FOR_INIT_TO_COMPLETE";
    case JSD_EL6001_SMS_READY_FOR_SERIAL_COMMUNICATION:         return "READY_FOR_SERIAL_COMMUNICATION";    

    default: assert(0); break;
  }
}

static char* jsd_el6001_transmit_sms_to_string(jsd_el6001_transmit_sms_t sms) {
  switch (sms)
  {
    case JSD_EL6001_TRANSMIT_SMS_WAITING_FOR_TRANSMIT_REQUEST_FROM_USER: return "WAITING_FOR_TRANSMIT_REQUEST_FROM_USER";
    case JSD_EL6001_TRANSMIT_SMS_WAITING_FOR_TRANSMIT_CONFIRMATION:      return "WAITING_FOR_TRANSMIT_CONFIRMATION_FROM_TERMINAL";

    default: assert(0); break;
  }
}

// Controlwords
static void jsd_el6001_set_controlword(jsd_t* self, uint16_t slave_id, uint16_t control_word) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_print_controlword("Setting controlword of: ", control_word, slave_id);
  self->slave_states[slave_id].el6001.controlword_user = control_word;
}

static uint16_t jsd_el6001_set_controlword_bit(const jsd_t* self, uint16_t slave_id, int bit) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(bit >= 0);
  assert(bit < JSD_EL6001_NUM_CONTROLWORD_BITS);

  return self->slave_states[slave_id].el6001.controlword_user | (1 << bit);
}

static uint16_t jsd_el6001_clear_controlword_bit(const jsd_t* self, uint16_t slave_id, int bit) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(bit >= 0);
  assert(bit < JSD_EL6001_NUM_CONTROLWORD_BITS);

  return self->slave_states[slave_id].el6001.controlword_user & ~(1 << bit);
}

static uint16_t jsd_el6001_toggle_controlword_bit(const jsd_t* self, uint16_t slave_id, int bit) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(bit >= 0);
  assert(bit < JSD_EL6001_NUM_CONTROLWORD_BITS);

  return self->slave_states[slave_id].el6001.controlword_user ^ (1 << bit);
}

// Statuswords
static bool jsd_el6001_statusword_bit_is_set(const jsd_t* self, uint16_t slave_id, int bit) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(bit >= 0);
  assert(bit < JSD_EL6001_NUM_STATUSWORD_BITS);

  return self->slave_states[slave_id].el6001.statusword & (1 << bit);  
}

static bool jsd_el6001_statusword_bit_went_high(const jsd_t* self, uint16_t slave_id, int bit) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(bit >= 0);
  assert(bit < JSD_EL6001_NUM_STATUSWORD_BITS);

  return( !(self->slave_states[slave_id].el6001.statusword_last & (1 << bit))
          && (self->slave_states[slave_id].el6001.statusword & (1 << bit)));
}

static bool jsd_el6001_statusword_bit_was_toggled(const jsd_t* self, uint16_t slave_id, int bit) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(bit >= 0);
  assert(bit < JSD_EL6001_NUM_STATUSWORD_BITS);

  return( (self->slave_states[slave_id].el6001.statusword_last & (1 << bit))
          != (self->slave_states[slave_id].el6001.statusword & (1 << bit)) );
}

// Fault check
static int jsd_el6001_check_for_faults(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_private_state_t* state  = &self->slave_states[slave_id].el6001;

  if (state->statusword != state->statusword_last)
  {
    if (jsd_el6001_statusword_bit_is_set(self, slave_id, JSD_EL6001_STATUSWORD_FIFO_BUFFER_FULL))
    {
      ERROR("EL6001[%d]: FIFO buffer is full", slave_id);
    }

    if (jsd_el6001_statusword_bit_is_set(self, slave_id, JSD_EL6001_STATUSWORD_PARITY_ERROR))
    {
      ERROR("EL6001[%d]: Parity error", slave_id);
    }

    if (jsd_el6001_statusword_bit_is_set(self, slave_id, JSD_EL6001_STATUSWORD_FRAMING_ERROR))
    {
      ERROR("EL6001[%d]: Framing error", slave_id);
    }

    if (jsd_el6001_statusword_bit_is_set(self, slave_id, JSD_EL6001_STATUSWORD_OVERRUN_ERROR))
    {
      ERROR("EL6001[%d]: Overrun error", slave_id);
    }
  }

  return 0;
}

// State Machine States
static int jsd_el6001_transition_sms(jsd_el6001_sms_t sms, jsd_el6001_sms_t* sms_ptr) {
  assert(sms_ptr != NULL);

  *sms_ptr = sms;
  MSG_DEBUG("Transitioning to %s state", jsd_el6001_sms_to_string(sms));

  return 0;
}

static int jsd_el6001_transition_transmit_sms(jsd_el6001_transmit_sms_t sms, jsd_el6001_transmit_sms_t* sms_ptr) {
  assert(sms_ptr != NULL);

  *sms_ptr = sms;
  MSG_DEBUG("Transitioning to %s state", jsd_el6001_transmit_sms_to_string(sms));

  return 0;
}

static int8_t jsd_el6001_compute_checksum(uint8_t data[], int num_bytes) {
  int i;
  int8_t checksum = 0;

  // sum each individual byte
  for (i = 0; i < num_bytes; ++i)
  {
    checksum += data[i];
  }

  return checksum;
}

//---------------------------------------------------------------------------//
// RECEIVE
//---------------------------------------------------------------------------//

static int jsd_el6001_receive_data(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_private_state_t* state = &self->slave_states[slave_id].el6001;

  int i;
  int num_bytes_received = 0;
  int8_t expected_checksum;
  int8_t checksum;
  bool ok = true;

  state->checksum_failed = false;

  // If terminal requests computer to receive data (ie. RECEIVE_REQUEST bit was toggled), indicating there's
  // newly received data, read the data bytes
  if (jsd_el6001_statusword_bit_was_toggled(self, slave_id, JSD_EL6001_STATUSWORD_RECEIVE_REQUEST))
  {
    MSG("EL6001[%d]: New data arrived at the terminal", slave_id);
    state->received_all_persistent_bytes = false;

    // Get number of data bytes received from the status word
    num_bytes_received = (state->statusword & JSD_EL6001_STATUSWORD_INPUT_LENGTH_BIT_MASK) >> JSD_EL6001_STATUSWORD_INPUT_LENGTH_0;
    MSG_DEBUG("EL6001[%d]: Terminal received %d bytes, waiting to accept reception from controller", slave_id, num_bytes_received);

    // assert number of bytes received is not above terminal capacity
    if(num_bytes_received > JSD_EL6001_NUM_DATA_BYTES)
    {
      ERROR("Terminal received more data than can handle");
      assert(0);
    }

    if (!state->received_first_byte_of_packet)
    {
      state->num_persistent_bytes_received = 0;
    }

    // Need to prevent IOOB on the persistent received bytes
    if(state->num_persistent_bytes_received + num_bytes_received > JSD_EL6001_MAX_NUM_DATA_BYTES)
    {
      ERROR("EL6001[%d]: Number of received bytes larger than persistence window. "
        "num_persistent_bytes_received = %d. "
        "num_bytes_received = %d. "
        "JSD_EL6001_MAX_NUM_DATA_BYTES = %d. ",
        slave_id,
        state->num_persistent_bytes_received,
        num_bytes_received,
        JSD_EL6001_MAX_NUM_DATA_BYTES);

      state->num_persistent_bytes_received = 0;
      state->received_first_byte_of_packet = false;
      ++state->read_errors;
    }

    // Store the received data bytes
    for (i = 0; i < num_bytes_received; ++i)
    {
      //state->received_bytes[] : Data is already populated while read PDO
      state->pub.persistent_received_bytes[state->num_persistent_bytes_received + i] = state->pub.received_bytes[i];
    }

    // increment number of persistent bytes by number received on this iteration
    state->num_persistent_bytes_received += num_bytes_received;

    // if first byte of persistent receipt wasn't received yet, set number of
    // bytes to receive from the first byte of the message
    if (!state->received_first_byte_of_packet)
    {
      MSG_DEBUG("EL6001[%d]: Received the first byte of a data packet", slave_id);
      if(state->use_first_byte_as_packet_length){
        /// Always set number of expected bytes based on the first byte of the packet
        state->expected_num_bytes_to_receive = (
          1 /*num_bytes byte*/
          + state->pub.received_bytes[0]
          + 1 /*checksum byte*/);
      }
      else{
        state->expected_num_bytes_to_receive = num_bytes_received;
      }

      MSG_DEBUG("EL6001[%d]: Expecting %d byte of data packet", slave_id, state->expected_num_bytes_to_receive);
      state->received_first_byte_of_packet = true;
    }

    // If there's a more than one whole expected packet, drop the earlier ones. Increment the error counter.
    if (state->num_persistent_bytes_received > 2 * state->expected_num_bytes_to_receive)
    {
      int packets_to_drop = floor(state->num_persistent_bytes_received / state->expected_num_bytes_to_receive) - 1;
      int bytes_to_drop = packets_to_drop * state->expected_num_bytes_to_receive;

      ERROR(
        "EL6001 id %d: "
        "Received %d persistent bytes, but expected_num_bytes_to_receive = %d. "
        "Dropping %d stale packets (%d bytes).",
        slave_id,
        state->num_persistent_bytes_received,
        state->expected_num_bytes_to_receive,
        packets_to_drop,
        bytes_to_drop);

      ++state->read_errors;

      // Dropping all packets before the latest whole packet
      // Shift the persistent bytes over by bytes_to_drop
      for (i = 0; i < (state->num_persistent_bytes_received - bytes_to_drop); ++i)
      {
        state->pub.persistent_received_bytes[i] = state->pub.persistent_received_bytes[i + bytes_to_drop];
      }
      state->num_persistent_bytes_received -= bytes_to_drop;
    }

    // If we received one whole packet, parse it. Checksum is last byte
    if (state->num_persistent_bytes_received >= state->expected_num_bytes_to_receive)
    {
      MSG_DEBUG("EL6001 id %d: Received all expected %d bytes of data packet", slave_id, state->expected_num_bytes_to_receive);

      if(state->use_last_byte_as_checksum){
        expected_checksum = state->pub.persistent_received_bytes[state->expected_num_bytes_to_receive - 1];
        checksum = jsd_el6001_compute_checksum(
          state->pub.persistent_received_bytes,
          state->expected_num_bytes_to_receive - 1/*checksum*/);
        if (expected_checksum != checksum)
        {
          ERROR("EL6001 id %d: Checksum invalid. Expected: %d, Computed: %d", slave_id, expected_checksum, checksum);
          state->checksum_failed = true;
          ++state->read_errors;        
        }
      }

      // No Partial Packet
      if (state->num_persistent_bytes_received == state->expected_num_bytes_to_receive)
      {
        // Received all expected bytes without errors. User must use this
        // before the next loop
        state->received_all_persistent_bytes = true;

        // reset first byte check
        state->received_first_byte_of_packet = false;
      }
      else
      {
        // Copy partial packet to front of queue
        ERROR(
          "EL6001 id %d: "
          "Full and partial packet received in one cycle. "
          "Received %d persistent bytes, but expected num bytes to receive "
          "is %d.",
          slave_id,
          state->num_persistent_bytes_received,
          state->expected_num_bytes_to_receive);

        ++state->read_errors;

        state->num_persistent_bytes_received -= state->expected_num_bytes_to_receive;
        for (i = 0; i < state->num_persistent_bytes_received; ++i)
        {
          state->pub.persistent_received_bytes[i] =
            state->pub.persistent_received_bytes[i + state->expected_num_bytes_to_receive];
        }
      }
    }

    // Tell terminal that data was received, so that new data can be received from the terminal
    jsd_el6001_set_controlword(self, slave_id, jsd_el6001_toggle_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_RECEIVE_ACCEPTED));

    MSG_DEBUG("EL6001[%d]: Data transferred with num bytes %d", slave_id, num_bytes_received);

    // Printout received data
    for(int i=0; i < num_bytes_received; i++){
      MSG_DEBUG("EL6001[%d]: Received data_in[%d]: %d", slave_id, i, state->pub.received_bytes[i]);
    }
  }

  // Set for user to know number of bytes received
  state->pub.num_bytes_received = num_bytes_received;

  return ok ? 0 : -1;
}

//---------------------------------------------------------------------------//
// TRANSMIT
//---------------------------------------------------------------------------//

static int jsd_el6001_transmit_data(jsd_t *self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_private_state_t* state = &self->slave_states[slave_id].el6001;

  switch (state->transmit_state)
  {
    case JSD_EL6001_TRANSMIT_SMS_WAITING_FOR_TRANSMIT_REQUEST_FROM_USER:
      if (state->user_requests_to_transmit_data)
      {
        MSG_DEBUG("Confirmed user request to transmit data, setting controlword to transmit request");
        // Tell terminal that data is available to transmit and then transition
        jsd_el6001_set_controlword(self, slave_id, jsd_el6001_toggle_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_TRANSMIT_REQUEST));        
        jsd_el6001_transition_transmit_sms(JSD_EL6001_TRANSMIT_SMS_WAITING_FOR_TRANSMIT_CONFIRMATION, &state->transmit_state);
        
        state->timer_start_sec = jsd_time_get_mono_time_sec();

        // Reset user transmit variable
        state->user_requests_to_transmit_data = false;
      }      
      break;

    case JSD_EL6001_TRANSMIT_SMS_WAITING_FOR_TRANSMIT_CONFIRMATION:
      {
        double time_since_last_transmit = jsd_time_get_mono_time_sec() - state->timer_start_sec;
        if (state->timeout_active && (time_since_last_transmit > state->timeout_sec))
        {
          ERROR("EL6001 id %d: Transmit request timed out after %f seconds while waiting for transmit confirmation.", slave_id, time_since_last_transmit);          
          jsd_el6001_transition_transmit_sms(JSD_EL6001_TRANSMIT_SMS_WAITING_FOR_TRANSMIT_REQUEST_FROM_USER, &state->transmit_state);
        }
        else if (jsd_el6001_statusword_bit_was_toggled(self, slave_id, JSD_EL6001_STATUSWORD_TRANSMIT_ACCEPTED))
        {
          // Beckhoff terminal transmitted data.
          MSG_DEBUG("Transmit request accepted, data transmit confirmed");

          if (state->pub.user_requests_to_transmit_data_persistently){
            // Start timeout timer
            state->timer_start_sec = jsd_time_get_mono_time_sec();
            // Request another transmit by toggling transmit_request
            jsd_el6001_set_controlword(self, slave_id, jsd_el6001_toggle_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_TRANSMIT_REQUEST));        
          }
          else if(!state->pub.user_requests_to_transmit_data_persistently){
            // Go back to waiting for another transmit request                              
            jsd_el6001_set_controlword(self, slave_id, self->slave_states[slave_id].el6001.controlword_user & 0x00FF); //clear data_out_length
            jsd_el6001_set_controlword(self, slave_id, jsd_el6001_clear_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_TRANSMIT_REQUEST));
            jsd_el6001_transition_transmit_sms(JSD_EL6001_TRANSMIT_SMS_WAITING_FOR_TRANSMIT_REQUEST_FROM_USER, &state->transmit_state);
          }

          //Increment the successful transmit counter
          if (state->autoincrement_byte >= 0)
          {
            jsd_el6001_set_transmit_data_8bits(self, slave_id, state->autoincrement_byte, state->pub.transmit_bytes[state->autoincrement_byte] + 1);            
          }
        }
      }
      break;

    default:
      assert(0);
      break;
  }

  return 0;
}

/****************************************************
 * Public functions
 ****************************************************/

const jsd_el6001_private_state_t* jsd_el6001_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);

  return &self->slave_states[slave_id].el6001;
}

void jsd_el6001_read(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);  
  
  jsd_el6001_private_state_t* state  = &self->slave_states[slave_id].el6001;         

  jsd_el6001_txpdo_data_t* txpdo =
      (jsd_el6001_txpdo_data_t*)self->ecx_context.slavelist[slave_id].inputs;

  state->statusword_last = state->statusword;
  state->statusword = txpdo->statusword;
  
  // TODO: check whether we can receive packed data as array
  for(int i = 0; i < JSD_EL6001_NUM_DATA_BYTES; i ++){
    state->pub.received_bytes[i] = txpdo->data_in[i];
  }
}

void jsd_el6001_write_PDO_data(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_private_state_t* state  = &self->slave_states[slave_id].el6001;         

  jsd_el6001_rxpdo_t* rxpdo =
      (jsd_el6001_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;      

  state->controlword_terminal_last = state->controlword_terminal;
  
  
  if(state->controlword_user_last != state->controlword_user){
    MSG_DEBUG("Writing a controlword as it has been updated");
    rxpdo->controlword = state->controlword_user;
    state->controlword_user_last = state->controlword_user;
  }

  if(state->controlword_user & (1 << JSD_EL6001_CONTROLWORD_TRANSMIT_REQUEST)){
    MSG_DEBUG("User request to transmit, populating data_out stream");
    uint8_t output_length = rxpdo->controlword >> JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_0;    
    for(uint8_t i = 0; i < output_length; i++){      
      rxpdo->data_out[i] = state->pub.transmit_bytes[i];
    }
  }  
}

void jsd_el6001_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_private_state_t* state  = &self->slave_states[slave_id].el6001;         

  // check for faults in statuswords
  jsd_el6001_check_for_faults(self, slave_id);

  // Main state machine
  switch (state->sms){
    case JSD_EL6001_SMS_INITING:
    {
      // Request Beckhoff terminal to initialize
      jsd_el6001_set_controlword(self, slave_id, jsd_el6001_set_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_INIT_REQUEST));
      // jsd_el6001_write_controlword(self, slave_id);
      jsd_el6001_transition_sms(JSD_EL6001_SMS_WAITING_FOR_TERMINAL_TO_INIT, &state->sms);
    }
    break;

    case JSD_EL6001_SMS_WAITING_FOR_TERMINAL_TO_INIT: // Waiting for Beckhoff terminal to initialize    
    {
      if (jsd_el6001_statusword_bit_went_high(self, slave_id, JSD_EL6001_STATUSWORD_INIT_ACCEPTED))
      {
        // Clear init request in order to make terminal ready for serial communication
        jsd_el6001_set_controlword(self, slave_id, jsd_el6001_clear_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_INIT_REQUEST));
        // jsd_el6001_write_controlword(self, slave_id);
        jsd_el6001_transition_sms(JSD_EL6001_SMS_READY_FOR_SERIAL_COMMUNICATION, &state->sms);
      }
    }
    break;

    case JSD_EL6001_SMS_READY_FOR_SERIAL_COMMUNICATION:
    {
      // Normal operation state machine state
      // receive serial data
      if (jsd_el6001_receive_data(self, slave_id) != 0)
      {
        assert(0);
        return;
      }

      // transmit user serial data
      jsd_el6001_transmit_data(self, slave_id);      
    }
    break;

    default:
      assert(0);
      break;
  }

  // Write slave state into RxPDO
  jsd_el6001_write_PDO_data(self, slave_id);
}

int jsd_el6001_set_transmit_data_8bits(jsd_t* self, uint16_t slave_id, int byte, uint8_t value) {  
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(byte < JSD_EL6001_NUM_DATA_BYTES);
  assert(byte >= 0);

  self->slave_states[slave_id].el6001.pub.transmit_bytes[byte] = value;

  return 0;
}

int jsd_el6001_set_transmit_data_payload(jsd_t* self, uint16_t slave_id, uint8_t* data_in, uint8_t data_len) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(data_len < JSD_EL6001_NUM_DATA_BYTES);  
  
  memcpy(self->slave_states[slave_id].el6001.pub.transmit_bytes, data_in, data_len*sizeof(uint8_t));  

  return 0; 
}

int jsd_el6001_set_baud_rate(jsd_t* self, uint16_t slave_id, jsd_el6001_baud_rate_t baud_rate) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(
    (baud_rate == JSD_EL6001_BAUD_RATE_2400)
    || (baud_rate == JSD_EL6001_BAUD_RATE_4800)
    || (baud_rate == JSD_EL6001_BAUD_RATE_9600)
    || (baud_rate == JSD_EL6001_BAUD_RATE_19200)
    || (baud_rate == JSD_EL6001_BAUD_RATE_38400)
    || (baud_rate == JSD_EL6001_BAUD_RATE_57600)
    || (baud_rate == JSD_EL6001_BAUD_RATE_115200)
    || (baud_rate == JSD_EL6001_BAUD_RATE_12000)
    || (baud_rate == JSD_EL6001_BAUD_RATE_14400));

  self->slave_states[slave_id].el6001.pub.baud_rate = baud_rate;

  return 0;
}

int jsd_el6001_request_transmit_data(jsd_t* self, uint16_t slave_id, int num_bytes_to_transmit) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  // Set number of output bytes by setting bits 8-15 of the control word to that number
  // Write the controlword so the terminal know number of bytes to send
  MSG("Clearing out any previous data output length");
  jsd_el6001_set_controlword(self, slave_id, self->slave_states[slave_id].el6001.controlword_user & 0x00FF);
  jsd_el6001_set_controlword(self, slave_id,
    self->slave_states[slave_id].el6001.controlword_user | (num_bytes_to_transmit << JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_0));

  // Set bool to advance transmit state machine
  self->slave_states[slave_id].el6001.user_requests_to_transmit_data = true;

  return 0;
}

int jsd_el6001_set_persistent_transmit_data(jsd_t* self, uint16_t slave_id, bool is_persistent) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  // Set bool to advance transmit state machine
  self->slave_states[slave_id].el6001.pub.user_requests_to_transmit_data_persistently = is_persistent;
  MSG("Setting transmit data as persistent: %s", is_persistent? "yes" : "no");

  return 0;
}

/****************************************************
 * Private functions
 ****************************************************/

bool jsd_el6001_init(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);
  assert(self->ecx_context.slavelist[slave_id].eep_man ==
         JSD_BECKHOFF_VENDOR_ID);

  ec_slavet* slaves = self->ecx_context.slavelist;
  ec_slavet* slave  = &slaves[slave_id];  

  slave->PO2SOconfigx = jsd_el6001_PO2SO_config;
  
  // Initialize main state machine
  self->slave_states[slave_id].el6001.sms = JSD_EL6001_SMS_INITING; // TODO: This used to be done in configure_sdos

  // Initialize transmit state machine
  self->slave_states[slave_id].el6001.transmit_state = JSD_EL6001_TRANSMIT_SMS_WAITING_FOR_TRANSMIT_REQUEST_FROM_USER;

  // Initialize timeout timer
  self->slave_states[slave_id].el6001.timer_start_sec = jsd_time_get_mono_time_sec();

  // Initialize packet config TODO: Link with fastcat config
  self->slave_states[slave_id].el6001.use_first_byte_as_packet_length = false;
  self->slave_states[slave_id].el6001.use_last_byte_as_checksum = false;

  // Initialize the autoincrement byte to -1 (none, or unknown)
  self->slave_states[slave_id].el6001.autoincrement_byte = -1;

  return true;
}

int jsd_el6001_PO2SO_config(ecx_contextt* ecx_context, uint16_t slave_id) {
  assert(ecx_context);
  assert(ecx_context->slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  // Since this function prototype is forced by SOEM, we have embedded a
  // reference to jsd.slave_configs within th ecx_context and extract it here
  jsd_slave_config_t* slave_configs =
      (jsd_slave_config_t*)ecx_context->userdata;

  jsd_slave_config_t* config = &slave_configs[slave_id];

  // Reset to factory default
  uint32_t reset_word = JSD_BECKHOFF_RESET_WORD;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, JSD_BECKHOFF_RESET_SDO,
                                  JSD_BECKHOFF_RESET_SUBIND, JSD_SDO_DATA_U32,
                                  &reset_word)) {
    return 0;
  }

  MSG("Configuring slave no: %u,  SII inferred name: %s", slave_id,
      ecx_context->slavelist[slave_id].name);
  MSG("\t Configured name: %s", config->name);

  MSG("\t baud rate: %s", jsd_el6001_baud_rate_strings[config->el6001.baud_rate]);

  uint32_t sdo_channel_index = 0x8000;

  // Set Baud Rate
  assert(config->el6001.baud_rate >= JSD_EL6001_BAUD_RATE_2400 && 
        config->el6001.baud_rate <= JSD_EL6001_BAUD_RATE_14400);
  uint8_t baud_rate = config->el6001.baud_rate;
  if (!jsd_sdo_set_param_blocking(ecx_context, slave_id, sdo_channel_index,
                                  0x11, JSD_SDO_DATA_U8, &baud_rate)) {
    return 0;
  } 

  config->PO2SO_success = true;
  return 1;
}

void jsd_el6001_set_autoincrement_byte(jsd_t* self, uint16_t slave_id, int byte) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(byte >= 0);
  assert(byte <= JSD_EL6001_MAX_NUM_DATA_BYTES);

  self->slave_states[slave_id].el6001.autoincrement_byte = byte;
}

