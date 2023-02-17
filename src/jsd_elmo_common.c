#include "jsd/jsd_elmo_common.h"

const char* jsd_elmo_state_machine_state_to_string(
    jsd_elmo_state_machine_state_t state) {
  switch (state) {
    case JSD_ELMO_STATE_MACHINE_STATE_NOT_READY_TO_SWITCH_ON:
      return "Not Ready to Switch On";
    case JSD_ELMO_STATE_MACHINE_STATE_SWITCH_ON_DISABLED:
      return "Switch On Disabled";
    case JSD_ELMO_STATE_MACHINE_STATE_READY_TO_SWITCH_ON:
      return "Ready to Switch On";
    case JSD_ELMO_STATE_MACHINE_STATE_SWITCHED_ON:
      return "Switched On";
    case JSD_ELMO_STATE_MACHINE_STATE_OPERATION_ENABLED:
      return "Operation Enabled";
    case JSD_ELMO_STATE_MACHINE_STATE_QUICK_STOP_ACTIVE:
      return "Quick Stop Active";
    case JSD_ELMO_STATE_MACHINE_STATE_FAULT_REACTION_ACTIVE:
      return "Fault Reaction Active";
    case JSD_ELMO_STATE_MACHINE_STATE_FAULT:
      return "Fault";
    default:
      return "Unknown State Machine State";
  }
}
