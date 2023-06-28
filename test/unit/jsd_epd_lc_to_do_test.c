#include <assert.h>

#include "jsd/jsd_epd_pub.h"

int main() {
  assert((jsd_epd_lc_to_do("AC") == 0x300C));
  assert((jsd_epd_lc_to_do("BP") == 0x303D));
  assert((jsd_epd_lc_to_do("CA") == 0x3052));
  assert((jsd_epd_lc_to_do("CL") == 0x305D));
  assert((jsd_epd_lc_to_do("CZ") == 0x306B));
  assert((jsd_epd_lc_to_do("DC") == 0x3078));
  assert((jsd_epd_lc_to_do("ER") == 0x30AB));
  assert((jsd_epd_lc_to_do("GS") == 0x30F4));
  assert((jsd_epd_lc_to_do("HL") == 0x3111));
  assert((jsd_epd_lc_to_do("LL") == 0x31A1));
  assert((jsd_epd_lc_to_do("MC") == 0x31BC));
  assert((jsd_epd_lc_to_do("PL") == 0x3231));
  assert((jsd_epd_lc_to_do("PX") == 0x323D));
  assert((jsd_epd_lc_to_do("SF") == 0x3297));
  assert((jsd_epd_lc_to_do("UM") == 0x32E6));
  assert((jsd_epd_lc_to_do("NADA") == 0x0000));

  MSG("Successful test");

  return 0;
}