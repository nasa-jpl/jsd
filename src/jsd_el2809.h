#ifndef JSD_EL2809_H
#define JSD_EL2809_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_el2809_pub.h"

/** @brief Initializes el2809
 *
 * @param self pointer JSD context
 * @param slave_id index of device on EtherCAT bus
 * @return true on success, false on failure
 */
bool jsd_el2809_init(jsd_t* self, uint16_t slave_id);

#ifdef __cplusplus
}
#endif

#endif
