#ifndef JSD_TYPES_H
#define JSD_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>
#include <stdbool.h>

#include "ethercat.h"
#include "jsd/jsd_ati_fts_types.h"
#include "jsd/jsd_egd_types.h"
#include "jsd/jsd_el1008_types.h"
#include "jsd/jsd_el2124_types.h"
#include "jsd/jsd_el2809_types.h"
#include "jsd/jsd_el3104_types.h"
#include "jsd/jsd_el3162_types.h"
#include "jsd/jsd_el3202_types.h"
#include "jsd/jsd_el3208_types.h"
#include "jsd/jsd_el3318_types.h"
#include "jsd/jsd_el3356_types.h"
#include "jsd/jsd_el3602_types.h"
#include "jsd/jsd_el4102_types.h"
#include "jsd/jsd_epd_nominal_types.h"
#include "jsd/jsd_epd_sil_types.h"
#include "jsd/jsd_error_cirq.h"
#include "jsd/jsd_ild1900_types.h"
#include "jsd/jsd_jed0101_types.h"
#include "jsd/jsd_jed0200_types.h"

typedef enum {
  JSD_DRIVER_TYPE_ATI_FTS = 0,
  JSD_DRIVER_TYPE_EGD,
  JSD_DRIVER_TYPE_EL1008,
  JSD_DRIVER_TYPE_EL2124,
  JSD_DRIVER_TYPE_EL2809,
  JSD_DRIVER_TYPE_EL3104,
  JSD_DRIVER_TYPE_EL3162,
  JSD_DRIVER_TYPE_EL3202,
  JSD_DRIVER_TYPE_EL3208,
  JSD_DRIVER_TYPE_EL3318,
  JSD_DRIVER_TYPE_EL3356,
  JSD_DRIVER_TYPE_EL3602,
  JSD_DRIVER_TYPE_EL4102,
  JSD_DRIVER_TYPE_EPD_NOMINAL,
  JSD_DRIVER_TYPE_EPD_SIL,
  JSD_DRIVER_TYPE_ILD1900,
  JSD_DRIVER_TYPE_JED0101,
  JSD_DRIVER_TYPE_JED0200,
} jsd_driver_type_t;

typedef struct {
  bool     configuration_active;
  jsd_driver_type_t driver_type;
  char     name[JSD_NAME_LEN];
  union {
    jsd_el1008_config_t  el1008;
    jsd_el3602_config_t  el3602;
    jsd_el3208_config_t  el3208;
    jsd_el2124_config_t  el2124;
    jsd_el2809_config_t  el2809;
    jsd_egd_config_t     egd;
    jsd_el3356_config_t  el3356;
    jsd_jed0101_config_t jed0101;
    jsd_jed0200_config_t jed0200;
    jsd_ati_fts_config_t ati_fts;
    jsd_el3104_config_t  el3104;
    jsd_el3202_config_t  el3202;
    jsd_el3318_config_t  el3318;
    jsd_el3162_config_t  el3162;
    jsd_el4102_config_t  el4102;
    jsd_ild1900_config_t ild1900;
    jsd_epd_nominal_config_t epd_nominal;
    jsd_epd_sil_config_t epd_sil;
  };
  bool PO2SO_success;  // reserved for internal use

} jsd_slave_config_t;

typedef struct {
  union {
    jsd_el1008_state_t      el1008;
    jsd_el3602_state_t      el3602;
    jsd_el3208_state_t      el3208;
    jsd_el2124_state_t      el2124;
    jsd_el2809_state_t      el2809;
    jsd_egd_private_state_t egd;
    jsd_el3356_state_t      el3356;
    jsd_jed0101_state_t     jed0101;
    jsd_jed0200_state_t     jed0200;
    jsd_ati_fts_state_t     ati_fts;
    jsd_el3104_state_t      el3104;
    jsd_el3202_state_t      el3202;
    jsd_el3318_state_t      el3318;
    jsd_el3162_state_t      el3162;
    jsd_el4102_state_t      el4102;
    jsd_ild1900_state_t     ild1900;
    jsd_epd_nominal_private_state_t epd_nominal;
    jsd_epd_sil_private_state_t epd_sil;
  };

  uint16_t num_async_sdo_requests; // reserved

} jsd_slave_state_t;

typedef union {
  int8_t   as_i8;
  int16_t  as_i16;
  int32_t  as_i32;
  int64_t  as_i64;
  float    as_float;
  double   as_double;
  uint8_t  as_u8;
  uint16_t as_u16;
  uint32_t as_u32;
  uint64_t as_u64;
} jsd_sdo_data_t;

typedef enum {
  JSD_SDO_DATA_UNSPECIFIED,
  JSD_SDO_DATA_I8,
  JSD_SDO_DATA_I16,
  JSD_SDO_DATA_I32,
  JSD_SDO_DATA_I64,
  JSD_SDO_DATA_FLOAT,
  JSD_SDO_DATA_DOUBLE,
  JSD_SDO_DATA_U8,
  JSD_SDO_DATA_U16,
  JSD_SDO_DATA_U32,
  JSD_SDO_DATA_U64,
} jsd_sdo_data_type_t;

typedef enum {
  JSD_SDO_REQ_TYPE_INVALID = 0,
  JSD_SDO_REQ_TYPE_READ,
  JSD_SDO_REQ_TYPE_WRITE,
} jsd_sdo_req_type_t;

typedef struct {
  // User parameters
  jsd_sdo_req_type_t  request_type;
  uint16_t            slave_id;
  uint16_t            sdo_index;
  uint8_t             sdo_subindex;
  jsd_sdo_data_t      data;
  jsd_sdo_data_type_t data_type;
  uint16_t            app_id;  // for application request tracking
  bool                success;  // response-only

  // Reserved parameters
  int wkc;  // debugging
} jsd_sdo_req_t;


typedef struct {
  jsd_sdo_req_t   buffer[JSD_SDO_REQ_CIRQ_LEN];
  uint16_t        r;
  uint16_t        w;
  char            name[JSD_NAME_LEN];
  pthread_mutex_t mutex;
} jsd_sdo_req_cirq_t;

/** * @brief main JSD context
 *
 * Contains list of slave configurations provided by user and internally updated
 * device states. The SOEM context is comes from SOEM Version 2
 * build configuration.
 */
typedef struct {
  jsd_slave_config_t slave_configs[EC_MAXSLAVE];
  jsd_slave_state_t  slave_states[EC_MAXSLAVE];
  jsd_error_cirq_t   slave_errors[EC_MAXSLAVE];

  ecx_contextt ecx_context;              ///< stores SOEM context
  char         IOmap[JSD_IOMAP_BYTES];   ///< IOmap contains read data from bus
  int          expected_wkc;             ///< Expected Working Counter
  int          wkc;                      ///< processdata Working Counter
  int          last_wkc;                 ///< the previous processdata wkc
  bool         init_complete;            ///< true after jsd_init(...)
  uint8_t      enable_autorecovery;      ///< enables autorecovery feature
  uint8_t      attempt_manual_recovery;  ///< one-time manual recovery attempt

  jsd_sdo_req_cirq_t jsd_sdo_req_cirq;
  jsd_sdo_req_cirq_t jsd_sdo_res_cirq;
  pthread_t          sdo_thread;
  pthread_cond_t     sdo_thread_cond;
  bool               sdo_join_flag;
  bool               raise_sdo_thread_cond;

} jsd_t;

#ifdef __cplusplus
}
#endif

#endif
