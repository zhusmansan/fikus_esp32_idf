/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "am2302_rmt.h"
#include "driver/gpio.h"
#include <esp_wifi.h>
#include <wifi_provisioning/manager.h>

// #include "wifi_manager.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "unity.h"

typedef struct AppState{
    float ambientTemperature;
    float ambientHumidity;
}AppState_t;

AppState_t appState ;

// DHT GPIO assignment
#define DHT22_DATA_PIN 22
#define DHT22_GRND_PIN 19

//Display pins
#define DISPLAY_SDA_PIN 26
#define DISPLAY_SCL_PIN 25
#define DISPLAY_VCC_PIN 32
#define DISPLAY_GND_PIN 33
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
static ssd1306_handle_t dev = NULL;

#define PERIFERIAL_POWER_STABILIZE_TIME 250

// Sensor poll period
#define SENSOR_POLL_PERIOD 1000 /*ms*/

//Sleep constants
#define TIME_TO_SLEEP 10
#define uS_TO_S_FACTOR 1000000
#define SLEEP_AFTER_INACTIVITY_PERIOD_MS 30000

void getSensorsValues(void * pvParameters)
{
  gpio_set_direction(DHT22_GRND_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(DHT22_GRND_PIN, 0);

  

  am2302_config_t am2302_config = {
      .gpio_num = DHT22_DATA_PIN,
  };
  am2302_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
  };
  am2302_handle_t sensor = NULL;
  
  am2302_new_sensor_rmt(&am2302_config, &rmt_config, &sensor);
  vTaskDelay(pdMS_TO_TICKS(PERIFERIAL_POWER_STABILIZE_TIME));
 
  while (1)
  {
    am2302_read_temp_humi(sensor, &(appState.ambientTemperature), &(appState.ambientHumidity));
    printf("Temperature: %.1f °C, Humidity: %.1f %%\n", appState.ambientTemperature, appState.ambientHumidity);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_PERIOD));
  }
}

esp_err_t ssd1306_show_signs(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);

    ssd1306_draw_bitmap(dev, 0, 2, &c_chSingal816[0], 14, 8);
    ssd1306_draw_bitmap(dev, 24, 2, &c_chBluetooth88[0], 8, 8);
    ssd1306_draw_bitmap(dev, 40, 2, &c_chMsg816[0], 16, 8);
    ssd1306_draw_bitmap(dev, 64, 2, &c_chGPRS88[0], 8, 8);
    ssd1306_draw_bitmap(dev, 90, 2, &c_chAlarm88[0], 8, 8);
    ssd1306_draw_bitmap(dev, 112, 2, &c_chBat816[0], 16, 8);
    char c [30];
    sprintf(c,"T:%.1fC, H:%.1f%%",appState.ambientTemperature, appState.ambientHumidity);
    // ssd1306_draw_string(dev, 0, 52, &c, 20, 0);
    ssd1306_draw_string(dev, 0, 16, &c, 12, 1);
    return ssd1306_refresh_gram(dev);
}

static void dev_ssd1306_initialization(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)DISPLAY_SDA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)DISPLAY_SCL_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
}

void displayTask(void * vpParameter){
  // gpio_set_direction(DHT22_GRND_PIN, GPIO_MODE_OUTPUT);
  // gpio_set_level(DHT22_GRND_PIN, 0);

  //Power up display
  //HACK! using gpio pins to power display
  gpio_set_direction(DISPLAY_GND_PIN , GPIO_MODE_OUTPUT);
  gpio_set_direction(DISPLAY_VCC_PIN , GPIO_MODE_OUTPUT);
  gpio_set_level(DISPLAY_GND_PIN , 0);
  gpio_set_level(DISPLAY_VCC_PIN , 1);

  vTaskDelay(pdMS_TO_TICKS(PERIFERIAL_POWER_STABILIZE_TIME));
  dev_ssd1306_initialization();
  while(true){
    ssd1306_show_signs(dev);
    vTaskDelay(1);
  }
} 

//this function will send ESP into deep sleep mode after 1 minute
// & will set up wakeup timer to wake each 5 seconds
void sleepDog(void * vpParameter){

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  vTaskDelay(pdMS_TO_TICKS(SLEEP_AFTER_INACTIVITY_PERIOD_MS));
  printf("Goto deep sleep!\n");
  esp_deep_sleep_start();
}

void app_main(void)
{
  
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  xTaskCreate(&getSensorsValues, "SensorsPoll_task", 4096, NULL, 5, NULL);

  if(ESP_SLEEP_WAKEUP_TIMER != wakeup_reason){
    printf("Wake up NOT BY TIMER\n");
    xTaskCreate(&displayTask, "Display_task", 4096, NULL, 5, NULL);
  
  }else{
    printf("Wake up by timer\n");

  }
  xTaskCreate(&sleepDog, "SleepDog_task", 2048, NULL, 5, NULL);
  

  while (true){
  //   printf("Hello world!\n");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
