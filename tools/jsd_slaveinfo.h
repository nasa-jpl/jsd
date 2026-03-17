#ifndef JSD_SLAVEINFO_H
#define JSD_SLAVEINFO_H

#include <stdbool.h>

void jsd_slaveinfo_print_usage(void);
int jsd_slaveinfo_run(const char* ifname, bool print_sdo, bool print_map);

#endif
