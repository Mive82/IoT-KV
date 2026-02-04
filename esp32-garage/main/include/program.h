#ifndef _MIVE_PROGRAM_H
#define _MIVE_PROGRAM_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "program_opaque.h"

#include "mqtt_client.h"
#include "driver/i2c_master.h"
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"

#include "garage.h"


struct mive_program_s
{
  QueueHandle_t main_queue;
  rc522_driver_handle_t nfc_driver;
  rc522_handle_t nfc_scanner;
  esp_mqtt_client_handle_t mqtt_client;
  mive_garage_t garage_handle;
  TaskHandle_t main_task_handle;

  rc522_picc_uid_t *nfc_uuids;
  uint32_t num_uuids;
  uint8_t nfc_state;
  uint8_t switch_state;
};

#endif // _MIVE_PROGRAM_H