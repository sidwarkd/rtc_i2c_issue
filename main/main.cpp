/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* ULP RISC-V RTC I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* This project is used to illustrate a bug with the RTC I2C peripheral in which
   the ENS210 sensor (and likely others as well) fail on the first I2C write attempt.

   This was originally reported in the following issue:
   https://github.com/espressif/esp-idf/issues/11037

   The suggested fix for the ACK checking portion of the issue was recommended by user
   @sudeep-mohanty as found here:
   https://github.com/espressif/esp-idf/issues/11037#issuecomment-1497163049

   Even with this change the issue persists with an occasional success. It is always
   the first write that fails in the failure case.
*/

// #include <stdio.h>
#include "esp_log.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ulp_riscv.h"
#include "ulp_riscv_i2c.h"
#include "ens210.h"

constexpr const char *TAG = "main";
/************************************************
 * RTC I2C utility APIs
 ************************************************/
static void init_i2c(void);
static void init_ens210(void);
static void read_ens210_data(void);
static void rtc_i2c_master_read_multiple_bytes(uint8_t reg_addr, uint8_t *data, size_t len);

uint32_t temperature_data = 0;
uint32_t humidity_data = 0;

constexpr ulp_riscv_i2c_cfg_t ULP_I2C_CFG = {
    .i2c_pin_cfg = {.sda_io_num = GPIO_NUM_3,
                    .scl_io_num = GPIO_NUM_2,
                    .sda_pullup_en = true,
                    .scl_pullup_en = true},
    .i2c_timing_cfg = {.scl_low_period = 1.4,
                       .scl_high_period = 0.3,
                       .sda_duty_period = 1,
                       .scl_start_period = 2,
                       .scl_stop_period = 1.3,
                       .i2c_trans_timeout = 20}};

/* This is a helper function to continuously read multiple registers sequentially. Per a comment by
@sudeep-mohanty found here https://github.com/espressif/esp-idf/issues/11037#issuecomment-1511516815

"For instance, you should be able to do a 2 byte read from the address 0x00 but cannot do a multi-byte read from 0x00 and 0x01 with one invocation of the API."

This function allows the equivalent of the common auto-increment behavior in I2C
*/
static void rtc_i2c_master_read_multiple_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        ulp_riscv_i2c_master_set_slave_reg_addr(reg_addr + i);
        ulp_riscv_i2c_master_read_from_device(data + i, 1);
    }
}

static void init_i2c(void)
{
    /* Configure RTC I2C */
    ESP_LOGI(TAG, "Initializing RTC I2C ... ");
    esp_err_t ret = ulp_riscv_i2c_master_init(&ULP_I2C_CFG);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "ERROR\nFailed to initialize RTC I2C. Aborting...");
        abort();
    }
    else
    {
        ESP_LOGI(TAG, "ok");
    }
}

static void init_ens210(void)
{
    ESP_LOGI(TAG, "Initializing ENS210 sensor ... ");
    uint8_t data_wr = 0;

    /* Configure I2C slave address */
    ulp_riscv_i2c_master_set_slave_addr(ENS210_ADDR);

    /* Reset the ENS210 sensor */
    // This is in a for loop because the first write fails most of the time but
    // subsequent writes are successful. This is a workaround for the issue but
    // should not be necessary.
    for (int i = 0; i < 3; i++)
    {
        ulp_riscv_i2c_master_set_slave_reg_addr(ENS210_SYS_CTRL);
        data_wr = ENS210_RESET;
        ESP_LOGI(TAG, "Writing to SYS_CTRL register");
        ulp_riscv_i2c_master_write_to_device(&data_wr, 1);

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    // Set the sensor run mode
    ulp_riscv_i2c_master_set_slave_reg_addr(ENS210_SENS_RUN);
    data_wr = 3; // continuous mode on both T and H
    ESP_LOGI(TAG, "Writing to SENS_RUN register");
    ulp_riscv_i2c_master_write_to_device(&data_wr, 1);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Enable low power mode
    ulp_riscv_i2c_master_set_slave_reg_addr(ENS210_SYS_CTRL);
    data_wr = ENS210_LOW_POWER_MODE_ENABLED; // low power mode enable
    ESP_LOGI(TAG, "Writing to SYS_CTRL register to enable low power mode");
    ulp_riscv_i2c_master_write_to_device(&data_wr, 1);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Start readings
    ulp_riscv_i2c_master_set_slave_reg_addr(ENS210_SENS_START);
    data_wr = 3; // start both T and H
    ESP_LOGI(TAG, "Writing to SENS_START register");
    ulp_riscv_i2c_master_write_to_device(&data_wr, 1);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    /* Confirm that the sensor is alive */
    ulp_riscv_i2c_master_set_slave_reg_addr(ENS210_SYS_STAT);
    uint8_t status = 0;
    ulp_riscv_i2c_master_read_from_device(&status, 1);

    ESP_LOGI(TAG, "Status: 0x%02X\n", status);

    /* Confirm that the sensor is alive */
    uint8_t part_id[2] = {0};
    rtc_i2c_master_read_multiple_bytes(ENS210_PART_ID, part_id, 2);

    uint16_t device_id = (part_id[1] << 8) | part_id[0];
    ESP_LOGI(TAG, "ENS210 Device ID: 0x%04X\n", device_id);

    if (device_id != DEVICE_ID_ENS210)
    {
        ESP_LOGE(TAG, "ERROR\nCannot communicate with ENS210 sensor!");
        abort();
    }
    else
    {
        ESP_LOGI(TAG, "ok");
    }
}

static void read_ens210_data(void)
{
    ESP_LOGI(TAG, "Reading temperature and humidity data from ENS210 sensor ...");

    /* Read temperature */
    uint8_t temp_data[3] = {0};
    rtc_i2c_master_read_multiple_bytes(ENS210_T_VAL, temp_data, 3);
    uint32_t temp_val = (temp_data[0] | (temp_data[1] << 8) | (temp_data[2] << 16)) & 0xffff;
    temperature_data = temp_val;

    /* Read humidity */
    uint8_t hum_data[3] = {0};
    rtc_i2c_master_read_multiple_bytes(ENS210_H_VAL, hum_data, 3);
    uint32_t hum_val = (hum_data[0] | (hum_data[1] << 8) | (hum_data[2] << 16)) & 0xffff;
    humidity_data = hum_val;
}

extern "C" void app_main(void)
{
    /* Initialize RTC I2C */
    init_i2c();

    while(true)
    {
        /* Initialize ENS210 sensor */
        // The init is under the while so the I2C write happens
        // over and over. This is to illustrate the issue and how
        // it only happens on the very first write.
        init_ens210();

        /* Add a delay to allow the sensor to finish first conversion */
        vTaskDelay(200 / portTICK_PERIOD_MS);

        /* Read initial temperature and humidity */
        read_ens210_data();

        float temperature = (float)(temperature_data / 64.0) - 273.150;
        float humidity = (float)(humidity_data / 512.0);
        ESP_LOGI(TAG, "Temperature = %.3f deg Celsius", temperature);
        ESP_LOGI(TAG, "Humidity = %.3f%%", humidity);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}