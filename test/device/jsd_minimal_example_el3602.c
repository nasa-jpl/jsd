#include "jsd/jsd_pub.h"

// device includes as necessary by application
#include "jsd/jsd_el3602_pub.h"

int main() {
  jsd_t* jsd = jsd_alloc();

  // configure the EL3602 device
  uint16_t slave_id = 2;  // 2nd device on this particular bus

  jsd_slave_config_t my_config = {0};

  my_config.el3602.range[0]  = JSD_EL3602_RANGE_10V;
  my_config.el3602.range[1]  = JSD_EL3602_RANGE_5V;
  my_config.el3602.filter[0] = JSD_BECKHOFF_FILTER_30000HZ;
  my_config.el3602.filter[1] = JSD_BECKHOFF_FILTER_30000HZ;

  snprintf(my_config.name, JSD_NAME_LEN, "my_el3602_device");
  my_config.configuration_active = true;
  my_config.product_code         = JSD_EL3602_PRODUCT_CODE;

  jsd_set_slave_config(jsd, slave_id, my_config);

  // Slave configuration must come before initialization
  if (!jsd_init(jsd, "eth9", 1)) {
    ERROR("Could not init jsd");
    return 0;
  }

  int i = 0;
  for (i = 0; i < 5e3; ++i) {  // loop for 5 seconds

    // Process slaves, including PDO exchange
    jsd_read(jsd, EC_TIMEOUTRET);
    jsd_el3602_read(jsd, slave_id);
    jsd_write(jsd);

    // Extract device state to use in application
    if (0 == i % 500) {  // only print out a few measurements in example

      const jsd_el3602_state_t* state = jsd_el3602_get_state(jsd, slave_id);

      MSG("Channel 1 volts: %lf Channel 2 volts: %lf", state->voltage[0],
          state->voltage[1]);
    }

    // Naive ~1000 hz loop rate
    usleep(1e3);
  }

  jsd_free(jsd);

  return 0;
}
