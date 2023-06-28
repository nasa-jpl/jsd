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

  SUCCESS("JSD is in Operational State");

  MSG("Closing SOEM and freeing memory...");
  jsd_free(jsd);

  SUCCESS("Done");

  return 0;
}
