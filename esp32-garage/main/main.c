#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "driver/i2c_master.h"
#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"
#include "esp_timer.h"

#include "include/wifi_handler.h"
#include "include/garage.h"
#include "include/events.h"
#include "include/program.h"

#include "ultrasonic.h"

static const char *TAG = "example";

#define MQTT_BROKER_URL "mqtt://192.168.82.15:1883"

// ==== Subscriber paths ====

// Got command to actuate garage door
#define MQTT_SWTICH_PATH "/garage/switch"
#define MQTT_SWITCH_STATE_PATH "/garage/switch/state"
// Command to start new NFC card registration
#define MQTT_REGISTER_NFC "/garage/auth/new"

// ==== Publisher paths ====

// State of garage door.
#define MQTT_STATE_PATH "/garage/state"
// State of NFC card registration.
#define MQTT_REGISTER_NFC_STATE "/garage/auth/state"
// Presence detection.
#define MQTT_PRESENCE_PATH "/garage/presence/distance"

// ==== NFC Stuff ====

#define RC522_SPI_BUS_GPIO_MISO    (19)
#define RC522_SPI_BUS_GPIO_MOSI    (23)
#define RC522_SPI_BUS_GPIO_SCLK    (18)
#define RC522_SPI_SCANNER_GPIO_SDA (5)
#define RC522_SCANNER_GPIO_RST     (4) // soft-reset

enum mive_nfc_state
{
  NFC_STATE_IDLE = 0,
  NFC_STATE_WAITING_FOR_CARD,
  NFC_STATE_REMOVE_CARD,
  NFC_STATE_SUCCESS,
  NFC_STATE_FAIL,
};

static char* mive_nfc_state_str[] = {
  [NFC_STATE_IDLE] = "IDLE",
  [NFC_STATE_WAITING_FOR_CARD] = "WAITING",
  [NFC_STATE_REMOVE_CARD] = "REMOVE",
  [NFC_STATE_SUCCESS] = "SUCCESS",
  [NFC_STATE_FAIL] = "FAIL",
};

#define NFC_MAX_UIDS 100
#define NFC_PARTITION_NAME "nvs_rfid"
#define NFC_STORAGE_NAMESPACE "uuids"

// ==== Ultrasonic Stuff ====

#define TRIGGER_GPIO 13
#define ECHO_GPIO 12
#define MAX_DISTANCE_CM 300

static rc522_spi_config_t driver_config = {
  .host_id = VSPI_HOST,
  .bus_config = &(spi_bus_config_t){
    .miso_io_num = RC522_SPI_BUS_GPIO_MISO,
    .mosi_io_num = RC522_SPI_BUS_GPIO_MOSI,
    .sclk_io_num = RC522_SPI_BUS_GPIO_SCLK,
    .data2_io_num = GPIO_NUM_NC,
    .data3_io_num = GPIO_NUM_NC,
    .data4_io_num = GPIO_NUM_NC,
    .data5_io_num = GPIO_NUM_NC,
    .data6_io_num = GPIO_NUM_NC,
    .data7_io_num = GPIO_NUM_NC,
  },
  .dev_config = {
    .spics_io_num = RC522_SPI_SCANNER_GPIO_SDA,
  },
  .rst_io_num = RC522_SCANNER_GPIO_RST,
};

esp_timer_handle_t timer_get_garage_state;
esp_timer_handle_t timer_send_garage_state;
esp_timer_handle_t timer_auth_idle;
esp_timer_handle_t timer_switch_reset;

static ultrasonic_sensor_t us_sensor = {
  .trigger_pin = TRIGGER_GPIO,
  .echo_pin = ECHO_GPIO
};

static uint8_t compare_uid(rc522_picc_uid_t uid1, rc522_picc_uid_t uid2);

static void timer_get_garage_state_callback(void* arg)
{
  mive_program_t* program = (mive_program_t*)arg;

  mive_event_t event = {.event_type = MIVE_EVENT_GET_GARAGE_INFO};
  xQueueSend(program->main_queue, &event, pdMS_TO_TICKS(50));
}

static void timer_send_garage_state_callback(void* arg)
{
  mive_program_t* program = (mive_program_t*)arg;

  mive_event_t event = {
    .event_type = MIVE_EVENT_MEASURE_DISTANCE
  };
  xQueueSend(program->main_queue, &event, pdMS_TO_TICKS(50));
}

static void timer_auth_idle_callback(void* arg)
{
  mive_program_t* program = (mive_program_t*)arg;

  program->nfc_state = NFC_STATE_IDLE;

  mive_event_t event = {
    .event_type = MIVE_EVENT_SEND_AUTH_STATE
  };
  xQueueSend(program->main_queue, &event, pdMS_TO_TICKS(50));
}

static void timer_switch_reset_callback(void* arg)
{
  mive_program_t* program = (mive_program_t*)arg;

  mive_event_t event = {
    .event_type = MIVE_EVENT_RESET_GARAGE_SWITCH
  };
  xQueueSend(program->main_queue, &event, pdMS_TO_TICKS(50));
}

static void nvs_write_uuids(mive_program_t* program)
{
  nvs_handle_t my_handle;
  esp_err_t err;

  err = nvs_open_from_partition(NFC_PARTITION_NAME, NFC_STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) nvs_open!", esp_err_to_name(err));
    return;
  }

  err = nvs_set_blob(my_handle, "uuid_data", program->nfc_uuids, NFC_MAX_UIDS * sizeof(*program->nfc_uuids));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) saving array data!", esp_err_to_name(err));
  }

  err = nvs_commit(my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) commiting data!", esp_err_to_name(err));
  }
  nvs_close(my_handle);
}

static esp_err_t nvs_load_uuids(mive_program_t* program)
{
  nvs_handle_t my_handle;
  esp_err_t err;

  size_t required_size = 0;
  err = nvs_open_from_partition(NFC_PARTITION_NAME, NFC_STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) nvs_open!", esp_err_to_name(err));
    return err;
  }

  err = nvs_get_blob(my_handle, "uuid_data", NULL, &required_size);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) nvs_get_blob!", esp_err_to_name(err));
  }

  if(required_size > (NFC_MAX_UIDS * sizeof(*program->nfc_uuids)))
  {
    ESP_LOGW(TAG, "Warning: Too much data. Got %d, expected max %d\n", required_size, (NFC_MAX_UIDS * sizeof(*program->nfc_uuids)));
    required_size = (NFC_MAX_UIDS * sizeof(*program->nfc_uuids));
  }

  err = nvs_get_blob(my_handle, "uuid_data", program->nfc_uuids, &required_size);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) nvs_get_blob!", esp_err_to_name(err));
    goto end;
  }

  program->num_uuids = required_size / sizeof(*program->nfc_uuids);

end:
  nvs_close(my_handle);
  return err;
}

static void add_uuid(mive_program_t* program, rc522_picc_uid_t* uuid)
{
  rc522_picc_uid_t* temp_uuid = NULL;

  for(uint32_t i = 0; i < NFC_MAX_UIDS; ++i)
  {
    temp_uuid = &program->nfc_uuids[i];
    if(temp_uuid->length == 0)
    {
      memcpy(temp_uuid->value, uuid->value, uuid->length);
      temp_uuid->length = uuid->length;

      break;
    }
  }

  return;
}

static void remove_uuid(mive_program_t* program, rc522_picc_uid_t* uuid)
{
  rc522_picc_uid_t* temp_uuid = NULL;

  for(uint32_t i = 0; i < NFC_MAX_UIDS; ++i)
  {
    temp_uuid = &program->nfc_uuids[i];
    if(compare_uid(*temp_uuid, *uuid))
    {
      memset(temp_uuid->value, 0, sizeof(temp_uuid->value));
      temp_uuid->length = 0;

      break;
    }
  }
  return;
}

static uint8_t check_uuid(mive_program_t* program, rc522_picc_uid_t* uuid)
{
  for(uint32_t i = 0; i < NFC_MAX_UIDS; ++i)
  {
    if(compare_uid(*uuid, program->nfc_uuids[i]))
    {
      return true;
    }
  }

  return false;
}

static uint8_t compare_uid(rc522_picc_uid_t uid1, rc522_picc_uid_t uid2)
{
  uint8_t retval = true;
  if(uid1.length == uid2.length)
  {
    for(int i = 0; i < uid1.length; ++i)
    {
      if(uid1.value[i] != uid2.value[i])
      {
        retval = false;
      }
    }
  }
  else{
    retval = false;
  }

  return retval;
}

static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
  mive_program_t *program = (mive_program_t*)arg;
  rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
  rc522_picc_t *picc = event->picc;

  if (picc->state == RC522_PICC_STATE_ACTIVE) {
    rc522_picc_print(picc);
    // If we aren't registering anything, do as normal
    if(program->nfc_state == NFC_STATE_IDLE)
    {
      if(check_uuid(program, &picc->uid))
      {
        mive_event_t m_event = {
          .event_type = MIVE_EVENT_START_GARAGE
        };

        xQueueSend(program->main_queue, &m_event, pdMS_TO_TICKS(10));
      }
    }
    else if(program->nfc_state == NFC_STATE_WAITING_FOR_CARD)
    {
      program->nfc_state = NFC_STATE_REMOVE_CARD;
      mive_event_t m_event = {
        .event_type = MIVE_EVENT_SEND_AUTH_STATE
      };
      xQueueSend(program->main_queue, &m_event, pdMS_TO_TICKS(10));
      add_uuid(program, &picc->uid);

      m_event.event_type = MIVE_EVENT_SAVE_UUID;
      xQueueSend(program->main_queue, &m_event, pdMS_TO_TICKS(10));
    }
  }
  else if (picc->state == RC522_PICC_STATE_IDLE && event->old_state >= RC522_PICC_STATE_ACTIVE) {
    ESP_LOGI(TAG, "Card has been removed");
    if(program->nfc_state == NFC_STATE_REMOVE_CARD)
    {
      program->nfc_state = NFC_STATE_SUCCESS;

      mive_event_t m_event = {
        .event_type = MIVE_EVENT_SEND_AUTH_STATE
      };
      xQueueSend(program->main_queue, &m_event, pdMS_TO_TICKS(10));
    }
  }
}

static void log_error_if_nonzero(const char *message, int error_code)
{
  if (error_code != 0) {
    ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
  }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  mive_program_t* program = (mive_program_t*)handler_args;
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  mive_event_t mive_event = {0};
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    mive_event.event_type = MIVE_EVENT_MQTT_CONNECTED;
    xQueueSend(program->main_queue, &mive_event, pdMS_TO_TICKS(100));

    esp_mqtt_client_subscribe(client, MQTT_SWTICH_PATH, 0);
    esp_mqtt_client_subscribe(client, MQTT_REGISTER_NFC, 0);

    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    if(strncasecmp(event->topic, MQTT_SWTICH_PATH, sizeof(MQTT_SWTICH_PATH) - 1) == 0)
    {
      mive_event.event_type = MIVE_EVENT_START_GARAGE;
      xQueueSend(program->main_queue, &mive_event, pdMS_TO_TICKS(10));
    } 
    else if(strncasecmp(event->topic, MQTT_REGISTER_NFC, sizeof(MQTT_REGISTER_NFC) - 1) == 0)
    {
      mive_event.event_type = MIVE_EVENT_REGISTER_CARD;
      xQueueSend(program->main_queue, &mive_event, pdMS_TO_TICKS(10));
    }
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
      log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
      log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
      log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
      ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

    }
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}

static void mqtt_app_start(mive_program_t* program)
{
  esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = MQTT_BROKER_URL,
  };

  program->mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
  esp_mqtt_client_register_event(program->mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, program);
  esp_mqtt_client_start(program->mqtt_client);
}

static void main_task(void* context)
{
  mive_program_t *program = (mive_program_t*)context;
  esp_err_t retval = ESP_OK;
  mive_event_t event = {0};
  enum garage_state_e garage_state = GARAGE_INVALID;
  enum garage_state_e new_garage_state = GARAGE_INVALID;
  uint32_t distance_cm = 0;

  const esp_timer_create_args_t get_garage_state_timer_args = {
    .callback = timer_get_garage_state_callback,
    /* name is optional, but may help identify the timer when debugging */
    .name = "get_garage_state_timer",
    .arg = program,
  };

  const esp_timer_create_args_t send_garage_state_timer_args = {
    .callback = timer_send_garage_state_callback,
    /* name is optional, but may help identify the timer when debugging */
    .name = "get_garage_state_timer",
    .arg = program,
  };

  const esp_timer_create_args_t auth_idle_timer_args = {
    .callback = timer_auth_idle_callback,
    /* name is optional, but may help identify the timer when debugging */
    .name = "auth_idle_timer",
    .arg = program,
  };

  const esp_timer_create_args_t switch_reset_timer_args = {
    .callback = timer_switch_reset_callback,
    .name = "switch_reset_timer",
    .arg = program,
  };

  ultrasonic_init(&us_sensor);

  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_create(&get_garage_state_timer_args, &timer_get_garage_state)); 
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_create(&send_garage_state_timer_args, &timer_send_garage_state));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_create(&auth_idle_timer_args, &timer_auth_idle));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_create(&switch_reset_timer_args, &timer_switch_reset));


  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_periodic(timer_get_garage_state, 250000));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_periodic(timer_send_garage_state, 1000000));

  while(1)
  {
    if (xQueueReceive(program->main_queue, &event, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
      switch (event.event_type)
      {
      case MIVE_EVENT_MQTT_CONNECTED:
        event.event_type = MIVE_EVENT_SEND_GARAGE_INFO;
        xQueueSend(program->main_queue, &event, pdMS_TO_TICKS(10));
        event.event_type = MIVE_EVENT_SEND_AUTH_STATE;
        xQueueSend(program->main_queue, &event, pdMS_TO_TICKS(10));
        break;
      case MIVE_EVENT_SEND_AUTH_STATE:
        esp_mqtt_client_publish(program->mqtt_client, MQTT_REGISTER_NFC_STATE, mive_nfc_state_str[program->nfc_state], 0, 1, 1);
        if(program->nfc_state >= NFC_STATE_SUCCESS)
        {
          ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(timer_auth_idle, 1000000));
        }
        break;
      case MIVE_EVENT_GET_GARAGE_INFO:
        new_garage_state = mive_garage_get_state(&program->garage_handle);
        if(new_garage_state != garage_state)
        {
          garage_state = new_garage_state;
          event.event_type = MIVE_EVENT_SEND_GARAGE_INFO;
          xQueueSend(program->main_queue, &event, pdMS_TO_TICKS(10));
        }
        break;
      case MIVE_EVENT_START_GARAGE:
        mive_garage_actuate(&program->garage_handle);
        event.event_type = MIVE_EVENT_GET_GARAGE_INFO;
        xQueueSend(program->main_queue, &event, pdMS_TO_TICKS(10));
        break;
      case MIVE_EVENT_SEND_GARAGE_INFO:
        esp_mqtt_client_publish(program->mqtt_client, MQTT_STATE_PATH, mive_garage_get_state_str(garage_state), 0, 1, 1);
        break;
      case MIVE_EVENT_MEASURE_DISTANCE:
        retval = ultrasonic_measure_cm(&us_sensor, MAX_DISTANCE_CM, &distance_cm);
        if (retval != ESP_OK)
        {
          esp_mqtt_client_publish(program->mqtt_client, MQTT_PRESENCE_PATH, "None", 0, 1, 1);
          printf("Error %d: ", retval);
          switch (retval)
          {
            case ESP_ERR_ULTRASONIC_PING:
              printf("Cannot ping (device is in invalid state)\n");
              break;
            case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
              printf("Ping timeout (no device found)\n");
              break;
            case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
              printf("Echo timeout (i.e. distance too big)\n");
              break;
            default:
              printf("%s\n", esp_err_to_name(retval));
          }

        }
        else
        {
          char buf[30] = {0};
          snprintf(buf, 29, "%ld", distance_cm);
          esp_mqtt_client_publish(program->mqtt_client, MQTT_PRESENCE_PATH, buf, 0, 1, 1);
          printf("Distance: %ld cm\n", distance_cm);
        }
        break;
      case MIVE_EVENT_REGISTER_CARD:
        program->nfc_state = NFC_STATE_WAITING_FOR_CARD;
        event.event_type = MIVE_EVENT_SEND_AUTH_STATE;
        xQueueSend(program->main_queue, &event, pdMS_TO_TICKS(10));
        break;
      case MIVE_EVENT_SAVE_UUID:
        nvs_write_uuids(program);
        break;
      case MIVE_EVENT_RESET_GARAGE_SWITCH:
        program->switch_state = 0;
        esp_mqtt_client_publish(program->mqtt_client, MQTT_SWITCH_STATE_PATH, "OFF", 0, 1, 1);
        break;
      default:
        break;
      }
    }
  }

  vTaskDelete(NULL);
}

void app_main(void)
{
  esp_err_t retval = ESP_OK;
  nvs_stats_t nvs_stats;

  i2c_master_bus_config_t bus_config = {
    .i2c_port = -1,
    .sda_io_num = 21,
    .scl_io_num = 22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };

  i2c_device_config_t dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x20,
    .scl_speed_hz = 100000,
  };

  mive_program_t *program = calloc(1, sizeof(*program));
  
  program->main_queue = xQueueCreate(20, sizeof(mive_event_t));
  program->nfc_state = NFC_STATE_IDLE;

  //Initialize NVS
  retval = nvs_flash_init();
  if (retval == ESP_ERR_NVS_NO_FREE_PAGES || retval == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    retval = nvs_flash_init();
  }
  ESP_ERROR_CHECK(retval);

  // Initialize NVS partition holding rfid keys
  retval = nvs_flash_init_partition(NFC_PARTITION_NAME);
  if (retval == ESP_ERR_NVS_NO_FREE_PAGES || retval == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase_partition(NFC_PARTITION_NAME));
    retval = nvs_flash_init_partition(NFC_PARTITION_NAME);
  }
  ESP_ERROR_CHECK(retval);

  program->nfc_uuids = calloc(1, NFC_MAX_UIDS * sizeof(*program->nfc_uuids));
  program->num_uuids = 0;

  nvs_load_uuids(program);

  retval = mive_garage_init(&program->garage_handle, &bus_config, &dev_config);

  ESP_ERROR_CHECK_WITHOUT_ABORT(retval);

  xTaskCreate(main_task, "MainTask", 15000, program, 15, &program->main_task_handle);

  rc522_spi_create(&driver_config, &program->nfc_driver);
  rc522_driver_install(program->nfc_driver);

  rc522_config_t scanner_config = {
    .driver = program->nfc_driver,
  };

  rc522_create(&scanner_config, &program->nfc_scanner);
  rc522_register_events(program->nfc_scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, program);
  rc522_start(program->nfc_scanner);

  wifi_init_sta(program);
  mqtt_app_start(program);

}
