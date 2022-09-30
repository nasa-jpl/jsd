#ifndef JSD_SDO_H
#define JSD_SDO_H

#include "jsd/jsd_sdo_pub.h"

#ifdef __cplusplus
extern "C" {
#endif

void jsd_sdo_req_cirq_init(jsd_sdo_req_cirq_t* self, const char* name);

jsd_sdo_req_t jsd_sdo_req_cirq_pop(jsd_sdo_req_cirq_t* self);

bool jsd_sdo_req_cirq_push(jsd_sdo_req_cirq_t* self, jsd_sdo_req_t req);

bool jsd_sdo_req_cirq_is_empty(jsd_sdo_req_cirq_t* self);

void* sdo_thread_loop(void* self);

int jsd_sdo_data_type_size(jsd_sdo_data_type_t type);

jsd_sdo_req_t 
  jsd_sdo_populate_request(uint16_t slave_id, 
                           uint16_t index,
                           uint8_t subindex, 
                           jsd_sdo_data_type_t data_type,
                           void* data,
                           jsd_sdo_req_type_t request_type,
                           uint16_t app_id);

void jsd_sdo_push_async_request(jsd_t* self, jsd_sdo_req_t request);

#ifdef __cplusplus
}
#endif

#endif
