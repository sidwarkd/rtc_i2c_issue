#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"
#include "../ulp_sensor.h"
#include "sensors/ens210.h"

// volatile float sensor_data[ULP_SENSOR_TOTAL] = {0.0};
volatile uint32_t sensor_data[ULP_SENSOR_TOTAL] = {0};
// volatile float sensor_max_thresholds[ULP_SENSOR_TOTAL] = {0.0};
// volatile float sensor_min_thresholds[ULP_SENSOR_TOTAL] = {0.0};

int main (void)
{
    // Read each sensor
    ens210_read_temp_and_humidity(&sensor_data[ENS210_TEMPERATURE_C], &sensor_data[ENS210_TEMPERATURE_F], &sensor_data[ENS210_HUMIDITY]);

    // Check if any sensor data is outside the threshold

    // Start any preparation for the next wakeup
    ens210_start_next_measurement();

    /* ulp_riscv_halt() is called automatically when main exits,
       main will be executed again at the next timeout period,
       according to ulp_set_wakeup_period()
     */
    return 0;
}