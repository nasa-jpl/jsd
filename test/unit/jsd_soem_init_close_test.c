#include <sys/types.h>

#include "jsd/jsd_pub.h"

int main(int argc, char* argv[]) {
  if (argc != 2) {
    ERROR("Expecting exactly 1 argument");
    MSG("Usage: jsd_el2124_test <ifname>");
    MSG("Example: $ jsd_soem_init_close_teset eth0");
    return 0;
  }

  jsd_t* jsd = jsd_alloc();

  if (!jsd_init(jsd, argv[1], 1)) {
    ERROR("Could not init jsd");
    return 0;
  }
  SUCCESS("JSD is initialized");

  // INIT
  if (!jsd_set_device_state(jsd, 0, EC_STATE_INIT, EC_TIMEOUTSTATE)) {
    return 0;
  }
  // PRE_OP
  if (!jsd_set_device_state(jsd, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE)) {
    return 0;
  }

  // START SLAVE CONFIGURATION

  // END SLAVE CONFIGURATION

  // SAFE_OP
  if (!jsd_set_device_state(jsd, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE)) {
    return 0;
  }
  // OP
  if (!jsd_set_device_state(jsd, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE)) {
    return 0;
  }

  SUCCESS("JSD is in Operational State");

  MSG("Closing SOEM and freeing memory...");
  jsd_free(jsd);

  SUCCESS("Done");

  return 0;
}
