#include <assert.h>

#include "jsd/jsd_epd_common.h"
#include "jsd/jsd_print.h"

int main() {
  assert((jsd_epd_lc_to_do_impl("AC") == 0x300C));
  assert((jsd_epd_lc_to_do_impl("BP") == 0x303D));
  assert((jsd_epd_lc_to_do_impl("CA") == 0x3052));
  assert((jsd_epd_lc_to_do_impl("CL") == 0x305D));
  assert((jsd_epd_lc_to_do_impl("CZ") == 0x306B));
  assert((jsd_epd_lc_to_do_impl("DC") == 0x3078));
  assert((jsd_epd_lc_to_do_impl("ER") == 0x30AB));
  assert((jsd_epd_lc_to_do_impl("GS") == 0x30F4));
  assert((jsd_epd_lc_to_do_impl("HL") == 0x3111));
  assert((jsd_epd_lc_to_do_impl("LL") == 0x31A1));
  assert((jsd_epd_lc_to_do_impl("MC") == 0x31BC));
  assert((jsd_epd_lc_to_do_impl("PL") == 0x3231));
  assert((jsd_epd_lc_to_do_impl("PX") == 0x323D));
  assert((jsd_epd_lc_to_do_impl("SF") == 0x3297));
  assert((jsd_epd_lc_to_do_impl("UM") == 0x32E6));
  assert((jsd_epd_lc_to_do_impl("NADA") == 0x0000));

  MSG("Successful test");

  return 0;
}