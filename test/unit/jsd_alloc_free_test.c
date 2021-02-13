#include "jsd/jsd_pub.h"

int main() {
  MSG("Allocating jsd_t");
  jsd_t* jsd = jsd_alloc();

  MSG("Deallocating jsd_t");
  jsd_free(jsd);

  MSG("Done");

  return 0;
}
