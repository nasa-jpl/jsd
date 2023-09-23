#ifndef JSD_EL6001_TYPES_H
#define JSD_EL6001_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_common_device_types.h"

#define JSD_EL6001_PRODUCT_CODE (uint32_t)0x17713052

#define JSD_EL6001_NUM_CHANNELS 1

#define JSD_EL6001_NUM_DATA_BYTES 22

#define JSD_EL6001_MAX_NUM_DATA_BYTES 100

#define JSD_EL6001_STATUSWORD_INPUT_LENGTH_BIT_MASK 0xFF00 ///< Bits 8-15 define the number of input bytes available for transfer from terminal to computer

typedef enum
{
  JSD_EL6001_CONTROLWORD_TRANSMIT_REQUEST = 0, ///< Toggle to notify terminal that DataOut bytes contain number of bytes indicated via
                                                 ///< OL bits. Terminal acknowledges receipt of data via toggle of SW0
  JSD_EL6001_CONTROLWORD_RECEIVE_ACCEPTED,     ///< Controller acknowledges receipt of data via toggle of this bit
  JSD_EL6001_CONTROLWORD_INIT_REQUEST,         ///< 1: Controller requests terminal to initialize. Terminal acknowledges completion by SW2
                                                 ///< 0: Controller once again request terminal to prepare for serial data exchange
  JSD_EL6001_CONTROLWORD_SEND_CONTINUOUS,      ///< Rising edge (0->1): Continuous sending of data from the FIFO buffer.
                                                 ///< Send buffer is filled (<= 128 bytes) by controller. Buffer content is sent with
                                                 ///< rising edge of this bit. Terminal acknowledges the data transfer to controller
                                                 ///< through setting of bit SW2. SW2 is cancelled with CW3
  JSD_EL6001_CONTROLWORD_RESERVED_0,
  JSD_EL6001_CONTROLWORD_RESERVED_1,
  JSD_EL6001_CONTROLWORD_RESERVED_2,
  JSD_EL6001_CONTROLWORD_RESERVED_3,
  JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_0,
  JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_1,
  JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_2,
  JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_3,
  JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_4,
  JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_5,
  JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_6,
  JSD_EL6001_CONTROLWORD_OUTPUT_LENGTH_7,
  JSD_EL6001_NUM_CONTROLWORD_BITS

} jsd_el6001_controlword_bit_t;


typedef enum
{
  JSD_EL6001_STATUSWORD_TRANSMIT_ACCEPTED = 0, ///< Toggle acknowledges receipt of data. Only now new data can be transmitted
                                                 ///< from controller to terminal.
  JSD_EL6001_STATUSWORD_RECEIVE_REQUEST,       ///< Toggle notifies the controller that DataIn bytes contain number of bytes in IL bits
                                                 ///< Controller must acknowledge receipt of data in control byte via toggling CW1.
                                                 ///< Only then new data can be transferred from terminal to controller.
  JSD_EL6001_STATUSWORD_INIT_ACCEPTED,         ///< 1: init completed by terminal. 0: terminal ready for serial data exchange.
  JSD_EL6001_STATUSWORD_FIFO_BUFFER_FULL,      ///< 1: all incoming data will be lost until buffer is not full
  JSD_EL6001_STATUSWORD_PARITY_ERROR,          ///< A parity error has occurred
  JSD_EL6001_STATUSWORD_FRAMING_ERROR,         ///< A framing error has occurred
  JSD_EL6001_STATUSWORD_OVERRUN_ERROR,         ///< An overrun error has occurred
  JSD_EL6001_STATUSWORD_RESERVED,
  JSD_EL6001_STATUSWORD_INPUT_LENGTH_0,        ///< = 8 (BIT08)
  JSD_EL6001_STATUSWORD_INPUT_LENGTH_1,
  JSD_EL6001_STATUSWORD_INPUT_LENGTH_2,
  JSD_EL6001_STATUSWORD_INPUT_LENGTH_3,
  JSD_EL6001_STATUSWORD_INPUT_LENGTH_4,
  JSD_EL6001_STATUSWORD_INPUT_LENGTH_5,
  JSD_EL6001_STATUSWORD_INPUT_LENGTH_6,
  JSD_EL6001_STATUSWORD_INPUT_LENGTH_7,        ///< = 15 (BIT15)
  JSD_EL6001_NUM_STATUSWORD_BITS               ///< = 16

} jsd_el6001_statusword_bit_t;

/**
 * @brief Set using object 0x8000:11 to one of the following values
 */
typedef enum
{
  JSD_EL6001_BAUD_RATE_2400   =  4,
  JSD_EL6001_BAUD_RATE_4800   =  5,
  JSD_EL6001_BAUD_RATE_9600   =  6,
  JSD_EL6001_BAUD_RATE_19200  =  7,
  JSD_EL6001_BAUD_RATE_38400  =  8,
  JSD_EL6001_BAUD_RATE_57600  =  9,
  JSD_EL6001_BAUD_RATE_115200 = 10,
  JSD_EL6001_BAUD_RATE_12000  = 14,
  JSD_EL6001_BAUD_RATE_14400  = 15,

} jsd_el6001_baud_rate_t;

/**
 * @brief Configuration struct for EL6001 device initialization
 */
typedef struct {
  jsd_el6001_baud_rate_t baud_rate;
  bool use_first_byte_as_packet_length;
  bool use_last_byte_as_checksum;
} jsd_el6001_config_t;

/**
 * @brief RxPDO struct used to set device command data in SOEM IOmap
 *
 * Note: struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {  
  uint16_t  controlword; //0x3003:0x01: 2byte
  uint8_t   data_out[JSD_EL6001_NUM_DATA_BYTES]; //0x3003:0x02 ~ 0x3003:0x17  
} jsd_el6001_rxpdo_t;

/**
 * @brief TxPDO struct used to read data in SOEM IOmap
 *
 * Note: struct order matters and must be packed.
 */
typedef struct __attribute__((__packed__)) {  
  uint16_t  statusword; //0x3103:01
  uint8_t   data_in[JSD_EL6001_NUM_DATA_BYTES]; //0x3103:0x02 ~ 0x3103:0x17  
} jsd_el6001_txpdo_data_t;

/**
 * @brief State Machine States needed for communication via serial RS232 interface 
 */
typedef enum
{
  JSD_EL6001_SMS_INITING = 0,
  JSD_EL6001_SMS_WAITING_FOR_TERMINAL_TO_INIT,
  JSD_EL6001_SMS_READY_FOR_SERIAL_COMMUNICATION,  
  JSD_EL6001_SMS_NUM_STATES

} jsd_el6001_sms_t;

/**
 * @brief Transmit State Machine States needed for communication via serial RS232 interface 
 */
typedef enum
{  
  JSD_EL6001_TRANSMIT_SMS_WAITING_FOR_TRANSMIT_REQUEST_FROM_USER = 0,
  JSD_EL6001_TRANSMIT_SMS_WAITING_FOR_TRANSMIT_CONFIRMATION,
  JSD_EL6001_TRANSMIT_SMS_NUM_STATES

} jsd_el6001_transmit_sms_t;

/**
 * @brief EL6001 state data
 */
typedef struct
{
  int baud_rate;
    
  uint8_t received_bytes[JSD_EL6001_NUM_DATA_BYTES]; // Data payload buffer received   
  uint8_t persistent_received_bytes[JSD_EL6001_MAX_NUM_DATA_BYTES];
  int num_bytes_received;

  uint8_t transmit_bytes[JSD_EL6001_NUM_DATA_BYTES];
    
  bool user_requests_to_transmit_data_persistently;  

} jsd_el6001_state_t;

typedef struct
{
  jsd_el6001_state_t pub;
  uint16_t statusword;
  uint16_t statusword_last;
  bool ready;  

  uint16_t controlword_terminal;
  uint16_t controlword_terminal_last;
  uint16_t controlword_user;
  uint16_t controlword_user_last;

  int read_errors;  
  
  uint8_t transmit_bytes_prev[JSD_EL6001_NUM_DATA_BYTES];
  bool transmit_data;
  bool user_requests_to_transmit_data;
  int autoincrement_byte;

  bool timeout_active;
  double timeout_sec;
  double timer_start_sec;
  bool timed_out;
  bool checksum_failed;

  // Persistent receiving of bytes
  int expected_num_bytes_to_receive;  
  int num_persistent_bytes_received;
  bool received_all_persistent_bytes;
  bool use_first_byte_as_packet_length;
  bool received_first_byte_of_packet;
  bool use_last_byte_as_checksum;
  
  jsd_el6001_sms_t sms;
  jsd_el6001_transmit_sms_t transmit_state;

} jsd_el6001_private_state_t;

#ifdef __cplusplus
}
#endif

#endif