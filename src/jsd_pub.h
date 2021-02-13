#ifndef JSD_PUB_H
#define JSD_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>

#include "ethercat.h"
#include "jsd/jsd_print.h"
#include "jsd/jsd_types.h"

/**
 * @brief Allocates JSD context
 *
 * The ecx_context struct is currently using some heap allocated memory
 *
 * @return Pointer to new JSD context
 */
jsd_t* jsd_alloc();

/**
 * @brief Sets user provided configuration
 *
 * All slaves should be configured before calling jsd_init(...)
 *
 * @param self pointer JSD context
 * @param slave_id index of slave on the bus
 * @param slave_config parameters for device initialization
 */
void jsd_set_slave_config(jsd_t* self, uint16_t slave_id,
                          jsd_slave_config_t slave_config);

/**
 * @brief Initializes SOEM on specified NIC
 *
 * Should be called after the user provides all slave configurations using
 * jsd_set_slave_config(...) function calls
 *
 * if enable_autorecovery is true, jsd_process may indeterministically attempt
 * recoveries if there are any workingcounter faults that disable the bus.
 * Enabling this feature is recommended but may overrun the loop period when
 * the auto recovery functions are being performed.
 *
 * @param self pointer JSD context
 * @param ifname specified name of NIC e.g. "eth0"
 * @param enable_autorecovery enables automatic recovery of lost devices
 * @return true on successful SOEM initialization
 */
bool jsd_init(jsd_t* self, const char* ifname, uint8_t enable_autorecovery);

/**
 * @brief Sets the state of the master within given time constraint
 *
 * Requesting the state for all slaves is achieved by setting slave_id=0
 *
 * A recommended timeout_us values is EC_TIMEOUTSTATE = 2e6us
 *
 * @param self pointer JSD context
 * @param slave_id id of device to change state. Use 0 to set entire bus state.
 * @param state master will assume this state.
 * @param timeout_us time in microseconds for given state switch
 * @return true on success
 * @return false on failure
 */
bool jsd_set_device_state(jsd_t* self, uint16_t slave_id, ec_state state,
                          int timeout_us);

/**
 * @brief Receive data from slave devices and store on local IOmap.
 *
 * @param self pointer JSD context
 * @param timeout_us in microseconds given to fetch frame on stack
 */
void jsd_read(jsd_t* self, int timeout_us);

/**
 * @brief Send data from slave devices and store on local IOmap.
 *
 * @param self pointer JSD context
 */
void jsd_write(jsd_t* self);

/**
 * @brief close library and free slave data array
 *
 * @param self pointer to main slave data array
 */
void jsd_free(jsd_t* self);

/**
 * @brief Get Device EtherCat State
 *
 * @param self pointer to JSD context
 * @param slave_id index of slave on the bus
 * @return current ethercat state
 */
ec_state jsd_get_device_state(jsd_t* self, uint16_t slave_id);

/**
 * @brief Attempt one-time manual bus recovery.
 * May be useful for expected hot-swaps or bus topology changes
 *
 * @param self pointer to JSD context
 */
void jsd_set_manual_recovery(jsd_t* self);

/**
 * @brief converts ec_state int to human-readable string
 *
 * @return ec_state as a string
 */
char* jsd_ec_state_to_string(ec_state state);

#ifdef __cplusplus
}
#endif

#endif
