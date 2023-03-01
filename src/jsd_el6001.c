#include "jsd/jsd_el6001.h"

#include <assert.h>
#include <math.h>

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
    case JSD_EL6001_SMS_WAITING_FOR_TRANSMIT_REQUEST_FROM_USER: return "WAITING_FOR_TRANSMIT_REQUEST_FROM_USER";
    case JSD_EL6001_SMS_WAITING_FOR_TRANSMIT_CONFIRMATION:      return "WAITING_FOR_TRANSMIT_CONFIRMATION_FROM_TERMINAL";

    default: assert(0); break;
  }
}

// Controlwords
static void jsd_el6001_set_controlword(jsd_t* self, uint16_t slave_id, uint16_t control_word) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_print_controlword("Setting control word of: ", control_word, slave_id);
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

static void jsd_el6001_write_controlword(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_state_t* state  = &self->slave_states[slave_id].el6001;
  jsd_el6001_rxpdo_t* rxpdo = (jsd_el6001_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;      

  jsd_el6001_print_controlword("Writing controlword to EtherCAT register of: ", state->controlword_user, slave_id);

  state->controlword_terminal_last = state->controlword_terminal;
  rxpdo->controlword = state->controlword_user;

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

  jsd_el6001_state_t* state  = &self->slave_states[slave_id].el6001;

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

  jsd_el6001_state_t* state = &self->slave_states[slave_id].el6001;

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
    state->received_all_persistent_bytes = false;

    // Get number of data bytes received from the status word
    num_bytes_received = (state->statusword & JSD_EL6001_STATUSWORD_INPUT_LENGTH_BIT_MASK) >> JSD_EL6001_STATUSWORD_INPUT_LENGTH_0;
    MSG_DEBUG("EL6001[%d]: num bytes received: %d", slave_id, num_bytes_received);

    // assert number of bytes received is not above terminal capacity
    if(num_bytes_received > JSD_EL6001_NUM_DATA_BYTES)
    {
      ERROR("Terminal received more data than can handle");
      assert(0);
    }

    if (!state->received_first_byte_of_msg)
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
      state->received_first_byte_of_msg = false;
      ++state->read_errors;
    }

    // Store the received data bytes
    for (i = 0; i < num_bytes_received; ++i)
    {
      state->persistent_received_bytes[state->num_persistent_bytes_received + i] = state->received_bytes[i];
    }

    // increment number of persistent bytes by number received on this iteration
    state->num_persistent_bytes_received += num_bytes_received;

    // if first byte of persistent receipt wasn't received yet, set number of
    // bytes to receive from the first byte of the message
    if (!state->received_first_byte_of_msg)
    {
      /// Always set number of expected bytes based on the first byte of the packet

      state->expected_num_bytes_to_receive = (
        1 /*num_bytes byte*/
        + state->received_bytes[0]
        + 1 /*checksum byte*/);

      state->received_first_byte_of_msg = true;
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
        state->persistent_received_bytes[i] = state->persistent_received_bytes[i + bytes_to_drop];
      }
      state->num_persistent_bytes_received -= bytes_to_drop;
    }

    // If we received one whole packet, parse it. Checksum is last byte
    if (state->num_persistent_bytes_received >= state->expected_num_bytes_to_receive)
    {
      MSG_DEBUG("EL6001 id %d: Received all expected %d bytes", slave_id, state->expected_num_bytes_to_receive);

      expected_checksum = state->persistent_received_bytes[state->expected_num_bytes_to_receive - 1];
      checksum = jsd_el6001_compute_checksum(
        state->persistent_received_bytes,
        state->expected_num_bytes_to_receive - 1/*checksum*/);
      if (expected_checksum != checksum)
      {
        ERROR("EL6001 id %d: Checksum invalid. Expected: %d, Computed: %d", slave_id, expected_checksum, checksum);
        state->checksum_failed = true;
        ++state->read_errors;        
      }

      // No Partial Packet
      if (state->num_persistent_bytes_received == state->expected_num_bytes_to_receive)
      {
        // Received all expected bytes without errors. User must use this
        // before the next loop
        state->received_all_persistent_bytes = true;

        // reset first byte check
        state->received_first_byte_of_msg = false;
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
          state->persistent_received_bytes[i] =
            state->persistent_received_bytes[i + state->expected_num_bytes_to_receive];
        }
      }
    }

    // Tell terminal that data was received, so that new data can be received from the terminal
    jsd_el6001_set_controlword(self, slave_id, jsd_el6001_toggle_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_RECEIVE_ACCEPTED));

    MSG_DEBUG("EL6001 id %d: num bytes received = %d", slave_id, num_bytes_received);
  }

  // Set for user to know number of bytes received
  state->num_bytes_received = num_bytes_received;

  return ok ? 0 : -1;
}

//---------------------------------------------------------------------------//
// TRANSMIT
//---------------------------------------------------------------------------//

static int jsd_el6001_transmit_data(jsd_t *self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_state_t* state = &self->slave_states[slave_id].el6001;

  switch (state->transmit_state)
  {
    case JSD_EL6001_SMS_WAITING_FOR_TRANSMIT_REQUEST_FROM_USER:
      if (state->user_requests_to_transmit_data)
      {
        // Tell terminal that data is available to transmit and then transition
        jsd_el6001_set_controlword(self, slave_id, jsd_el6001_toggle_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_TRANSMIT_REQUEST));

        jsd_el6001_transition_sms(JSD_EL6001_SMS_WAITING_FOR_TRANSMIT_CONFIRMATION, &state->transmit_state);

        state->timer_start_sec = jsd_time_get_mono_time_sec();

        // Reset user transmit variable
        state->user_requests_to_transmit_data = false;
      }
      else if (state->user_requests_to_transmit_data_persistently)
      {
        if (jsd_el6001_all_persistent_data_was_received(self, slave_id))
        {
          // Tell terminal that data is available to transmit and then transition
          jsd_el6001_set_controlword(self, slave_id, jsd_el6001_toggle_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_TRANSMIT_REQUEST));

          jsd_el6001_transition_sms(JSD_EL6001_SMS_WAITING_FOR_TRANSMIT_CONFIRMATION, &state->transmit_state);

          state->timer_start_sec = jsd_time_get_mono_time_sec();
        }
        else
        {
          double time_since_last_transmit = jsd_time_get_mono_time_sec() - state->timer_start_sec;
          if (state->timeout_active && (time_since_last_transmit > state->timeout_sec))
          {
            ERROR("EL6001 id %d: Transmit timed out after %f seconds while waiting for persistent data.", slave_id, time_since_last_transmit);
            ERROR("EL6001 id %d: Clearing persistent data buffer in order to transmit fresh data in the next cycle.", slave_id);
            state->num_persistent_bytes_received = 0;
            state->received_all_persistent_bytes = 1;
            state->received_first_byte_of_msg = false;
          }
        }
      }
      break;

    case JSD_EL6001_SMS_WAITING_FOR_TRANSMIT_CONFIRMATION:
      {
        double time_since_last_transmit = jsd_time_get_mono_time_sec() - state->timer_start_sec;
        if (state->timeout_active && (time_since_last_transmit > state->timeout_sec))
        {
          ERROR("EL6001 id %d: Transmit timed out after %f seconds while waiting for transmit confirmation.", slave_id, time_since_last_transmit);
          jsd_el6001_transition_sms(JSD_EL6001_SMS_WAITING_FOR_TRANSMIT_REQUEST_FROM_USER, &state->transmit_state);
        }
        else if (jsd_el6001_statusword_bit_was_toggled(self, slave_id, JSD_EL6001_STATUSWORD_TRANSMIT_ACCEPTED))
        {
          // Beckhoff terminal transmitted data.
          MSG_DEBUG("Transmit accepted");

          // Go back to waiting for another transmit request
          jsd_el6001_transition_sms(JSD_EL6001_SMS_WAITING_FOR_TRANSMIT_REQUEST_FROM_USER, &state->transmit_state);

          // Start timeout timer
          state->timer_start_sec = jsd_time_get_mono_time_sec();

          //Increment the successful transmit counter
          if (state->autoincrement_byte >= 0)
          {
            jsd_el6001_set_transmit_data_8bits(self, slave_id, state->autoincrement_byte, state->transmit_bytes[state->autoincrement_byte] + 1);
            // write into RxPDO
            jsd_el6001_write_PDO_data(self, slave_id);
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

const jsd_el6001_state_t* jsd_el6001_get_state(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);

  return &self->slave_states[slave_id].el6001;
}

void jsd_el6001_read_PDO_data(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_state_t* state  = &self->slave_states[slave_id].el6001;         

  jsd_el6001_txpdo_data_t* txpdo =
      (jsd_el6001_txpdo_data_t*)self->ecx_context.slavelist[slave_id].inputs;

  // jsd_el3602_rxpdo_t* rxpdo =
  //     (jsd_el3602_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;      

  // state->control_word_terminal_last = state->control_word_terminal;
  // state->control_word = rxpdo->control_word;

  state->statusword_last = state->statusword;
  state->statusword = txpdo->statusword;
  
  // TODO: check whether we can receive packed data as array
  state->received_bytes[0] = txpdo->data_in_0;
  state->received_bytes[1] = txpdo->data_in_1;
  state->received_bytes[2] = txpdo->data_in_2;
  state->received_bytes[3] = txpdo->data_in_3;
  state->received_bytes[4] = txpdo->data_in_4;
  state->received_bytes[5] = txpdo->data_in_5;
  state->received_bytes[6] = txpdo->data_in_6;
  state->received_bytes[7] = txpdo->data_in_7;
  state->received_bytes[8] = txpdo->data_in_8;
  state->received_bytes[9] = txpdo->data_in_9;
  state->received_bytes[10] = txpdo->data_in_10;
  state->received_bytes[11] = txpdo->data_in_11;  
  state->received_bytes[12] = txpdo->data_in_12;
  state->received_bytes[13] = txpdo->data_in_13;
  state->received_bytes[14] = txpdo->data_in_14;
  state->received_bytes[15] = txpdo->data_in_15;
  state->received_bytes[16] = txpdo->data_in_16;
  state->received_bytes[17] = txpdo->data_in_17;
  state->received_bytes[18] = txpdo->data_in_18;
  state->received_bytes[19] = txpdo->data_in_19;
  state->received_bytes[20] = txpdo->data_in_20;
  state->received_bytes[21] = txpdo->data_in_21;
}

void jsd_el6001_write_PDO_data(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_state_t* state  = &self->slave_states[slave_id].el6001;         

  jsd_el6001_rxpdo_t* rxpdo =
      (jsd_el6001_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;      

  state->controlword_terminal_last = state->controlword_terminal;
  rxpdo->controlword = state->controlword_user;
  
  // TODO: check whether we can receive packed data as array
  rxpdo->data_out_0 = state->transmit_bytes[0];
  rxpdo->data_out_1 = state->transmit_bytes[1];
  rxpdo->data_out_2 = state->transmit_bytes[2];
  rxpdo->data_out_3 = state->transmit_bytes[3];
  rxpdo->data_out_4 = state->transmit_bytes[4];
  rxpdo->data_out_5 = state->transmit_bytes[5];
  rxpdo->data_out_6 = state->transmit_bytes[6];
  rxpdo->data_out_7 = state->transmit_bytes[7];
  rxpdo->data_out_8 = state->transmit_bytes[8];
  rxpdo->data_out_9 = state->transmit_bytes[9];
  rxpdo->data_out_10 = state->transmit_bytes[10];
  rxpdo->data_out_11 = state->transmit_bytes[11];
  rxpdo->data_out_12 = state->transmit_bytes[12];
  rxpdo->data_out_13 = state->transmit_bytes[13];
  rxpdo->data_out_14 = state->transmit_bytes[14];
  rxpdo->data_out_15 = state->transmit_bytes[15];
  rxpdo->data_out_16 = state->transmit_bytes[16];
  rxpdo->data_out_17 = state->transmit_bytes[17];
  rxpdo->data_out_18 = state->transmit_bytes[18];
  rxpdo->data_out_19 = state->transmit_bytes[19];
  rxpdo->data_out_20 = state->transmit_bytes[20];
  rxpdo->data_out_21 = state->transmit_bytes[21];
}

void jsd_el6001_process(jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id ==
         JSD_EL6001_PRODUCT_CODE);

  jsd_el6001_state_t* state  = &self->slave_states[slave_id].el6001;         
  
  // jsd_el6001_rxpdo_t* rxpdo = (jsd_el6001_rxpdo_t*)self->ecx_context.slavelist[slave_id].outputs;

  // read PDOs, updates statusword
  jsd_el6001_read_PDO_data(self, slave_id);

  // check for faults in statuswords
  jsd_el6001_check_for_faults(self, slave_id);

  // Main state machine
  switch (state->sms){
    case JSD_EL6001_SMS_INITING:
    {
      // Request Beckhoff terminal to initialize
      jsd_el6001_set_controlword(self, slave_id, jsd_el6001_set_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_INIT_REQUEST));
      jsd_el6001_transition_sms(JSD_EL6001_SMS_WAITING_FOR_TERMINAL_TO_INIT, &state->sms);
    }
    break;

    case JSD_EL6001_SMS_WAITING_FOR_TERMINAL_TO_INIT: // Waiting for Beckhoff terminal to initialize    
    {
      if (jsd_el6001_statusword_bit_went_high(self, slave_id, JSD_EL6001_STATUSWORD_INIT_ACCEPTED))
      {
        // Clear init request in order to make terminal ready for serial communication
        jsd_el6001_set_controlword(self, slave_id, jsd_el6001_clear_controlword_bit(self, slave_id, JSD_EL6001_CONTROLWORD_INIT_REQUEST));
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

      // If any of the bytes to transmit have changed, write them to the bus
      size_t byte_ndx;
      for (byte_ndx = 0; byte_ndx < JSD_EL6001_NUM_DATA_BYTES; byte_ndx++)
      {
        if (state->transmit_bytes[byte_ndx] != state->transmit_bytes_prev[byte_ndx])
        {
          // rxpdo->data_out_0 = state->transmit_bytes[byte_ndx];
          // Write to RxPDO
          // EC_WRITE_U8(
          //   self->domain_pd_output + self->offsets_output[id][JSD_EL6001_DATA_OUT_BYTE_00 + byte_ndx],
          //   state->transmit_bytes[byte_ndx]);
        }
        state->transmit_bytes_prev[byte_ndx] = state->transmit_bytes[byte_ndx];
      }

      // transmit user serial data
      jsd_el6001_transmit_data(self, slave_id);
    }
    break;

    default:
      assert(0);
      break;
  }
  
  // If the controlword has changed, write it and update the "last" (most recent) value.
  if (state->controlword_user != state->controlword_user_last)
  {
    jsd_el6001_write_controlword(self, slave_id);
    state->controlword_user_last = state->controlword_user;
  }

}

bool jsd_el6001_all_persistent_data_was_received(const jsd_t* self, uint16_t slave_id) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);

  return self->slave_states[slave_id].el6001.received_all_persistent_bytes;
}

int jsd_el6001_set_transmit_data_8bits(jsd_t* self, uint16_t slave_id, int byte, uint8_t value) {
  assert(self);
  assert(self->ecx_context.slavelist[slave_id].eep_id == JSD_EL6001_PRODUCT_CODE);
  assert(byte < JSD_EL6001_DATA_OUT_BYTE_21);
  assert(byte >= 0);

  self->slave_states[slave_id].el6001.transmit_bytes[byte] = value;

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

  self->slave_states[slave_id].el6001.baud_rate = baud_rate;

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

  jsd_slave_config_t* config = &self->slave_configs[slave_id];
  jsd_el6001_state_t* state  = &self->slave_states[slave_id].el6001;     

  // TODO No PO2SO callback for 6001 devices, so set the success flag now
  config->PO2SO_success = true;

  state->sms = JSD_EL6001_SMS_INITING; // TODO: This used to be done in configure_sdos

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

