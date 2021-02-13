#ifndef JSD_SDO_PUB_H
#define JSD_SDO_PUB_H

#include "jsd/jsd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void jsd_sdo_set_param_async(jsd_t* self, uint16_t slave_id, uint16_t index,
                             uint8_t subindex, jsd_sdo_data_type_t data_type,
                             void* param_in);

void jsd_sdo_get_param_async(jsd_t* self, uint16_t slave_id, uint16_t index,
                             uint8_t subindex, jsd_sdo_data_type_t data_type);

bool jsd_sdo_set_param_blocking(ecx_contextt* ecx_context, uint16_t slave_id,
                                uint16_t index, uint8_t subindex,
                                jsd_sdo_data_type_t data_type, void* param_in);

bool jsd_sdo_get_param_blocking(ecx_contextt* ecx_context, uint16_t slave_id,
                                uint16_t index, uint8_t subindex,
                                jsd_sdo_data_type_t data_type, void* param_out);

bool jsd_sdo_set_ca_param_blocking(ecx_contextt* ecx_context, uint16_t slave_id,
                                   uint16_t index, uint8_t subindex,
                                   int param_size, void* param_in);

bool jsd_sdo_get_ca_param_blocking(ecx_contextt* ecx_context, uint16_t slave_id,
                                   uint16_t index, uint8_t subindex,
                                   int* param_size_in_out, void* param_out);

#ifdef __cplusplus
}
#endif

#endif
