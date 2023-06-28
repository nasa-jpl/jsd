#ifndef JSD_ELMO_COMMON_H
#define JSD_ELMO_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "jsd/jsd_elmo_common_types.h"

/**
 * @brief Converts jsd_elmo_state_machine_state_t label to string
 *
 * @return string representation of enumeration label
 */
const char* jsd_elmo_state_machine_state_to_string(
    jsd_elmo_state_machine_state_t state);

#ifdef __cplusplus
}
#endif

#endif