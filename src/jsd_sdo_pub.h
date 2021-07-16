#ifndef JSD_SDO_PUB_H
#define JSD_SDO_PUB_H

#include "jsd/jsd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief A request to set a COE parameter after intialization using the 
 * background SDO thread
 *
 * A most the background thread exchanges one device SDO request each cycle. 
 * Confirmation of the parameter change will be printed to STDOUT. 
 * Device specific responses are possible with feature requests.
 *
 * @param self pointer to jsd context
 * @param slave_id The id of the slave 
 * @param index the COE parameter index value
 * @param subindex the COE parameter subindex value
 * @param data_type the type of the COE parameter e.g. U16
 * @param param_in raw pointer to a value of data_type type
 * @return void
 */
void jsd_sdo_set_param_async(jsd_t* self, uint16_t slave_id, uint16_t index,
                             uint8_t subindex, jsd_sdo_data_type_t data_type,
                             void* param_in);

/** A request to retrieve a COE parameter after initialization using 
 * the background SDO thread
 *
 * The default SDO response handler will print the parameter value to STDOUT.
 * Device specific responses are possible with feature requests.
 * 
 * @param self pointer to jsd context
 * @param slave_id The id of the slave 
 * @param index the COE parameter index value
 * @param subindex the COE parameter subindex value
 * @param data_type the type of the COE parameter e.g. U16
 * @return void
 */
void jsd_sdo_get_param_async(jsd_t* self, uint16_t slave_id, uint16_t index,
                             uint8_t subindex, jsd_sdo_data_type_t data_type);


/** @brief A blocking request to set a COE parameter
 *
 * Only to be used during PO2SO callbacks or before transitioning to OPERATIONAL
 *
 * @param ecx_context Pointer to SOEM Ethercat bus context
 * @param slave_id The id of the slave 
 * @param index the COE parameter index value
 * @param subindex the COE parameter subindex value
 * @param data_type the type of the COE parameter e.g. U16
 * @param param_in raw pointer to a value of data_type 
 * @return bool true if parameter was updated, false if parameter could not be set
 */
bool jsd_sdo_set_param_blocking(ecx_contextt* ecx_context, uint16_t slave_id,
                                uint16_t index, uint8_t subindex,
                                jsd_sdo_data_type_t data_type, void* param_in);

/** A blocking request to get a COE parameter
 *
 * Only to be used during PO2SO callbacks or before transitioning to OPERATIONAL
 *
 * @param ecx_context Pointer to SOEM Ethercat bus context
 * @param slave_id The id of the slave 
 * @param index the COE parameter index value
 * @param subindex the COE parameter subindex value
 * @param data_type the type of the COE parameter  e.g. U16
 * @param param_out raw pointer to the retrieved parameter value
 * @return bool true if parameter was retrieved, false if parameter could not be retrieved 
 */
bool jsd_sdo_get_param_blocking(ecx_contextt* ecx_context, uint16_t slave_id,
                                uint16_t index, uint8_t subindex,
                                jsd_sdo_data_type_t data_type, void* param_out);


/** @brief A blocking request to set a COE parameter through Complete Access (CA)
 *
 * Only to be used during PO2SO callbacks or before transitioning to OPERATIONAL
 *
 * @param ecx_context Pointer to SOEM Ethercat bus context
 * @param slave_id The id of the slave 
 * @param index the COE parameter index value
 * @param subindex the COE parameter subindex value
 * @param param_size size of CA data to be written
 * @param param_in CA data to be written
 * @return bool true if parameter was updated, false if parameter could not be set
 */
bool jsd_sdo_set_ca_param_blocking(ecx_contextt* ecx_context, uint16_t slave_id,
                                   uint16_t index, uint8_t subindex,
                                   int param_size, void* param_in);

/** A blocking request to get a COE parameter through Complete Access (CA)
 *
 * Only to be used during PO2SO callbacks or before transitioning to OPERATIONAL
 *
 * @param ecx_context Pointer to SOEM Ethercat bus context
 * @param slave_id The id of the slave 
 * @param index the COE parameter index value
 * @param subindex the COE parameter subindex value
 * @param param_size_in_out input as the size of the param_out buffer, returns the number
 *   bytes written to param_out upon success
 * @param param_out raw pointer to the retrieved parameter value
 * @return bool true if parameter was retrieved, false if parameter could not be retrieved 
 */
bool jsd_sdo_get_ca_param_blocking(ecx_contextt* ecx_context, uint16_t slave_id,
                                   uint16_t index, uint8_t subindex,
                                   int* param_size_in_out, void* param_out);

#ifdef __cplusplus
}
#endif

#endif
