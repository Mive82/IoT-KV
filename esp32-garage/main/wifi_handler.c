#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_netif_sntp.h"

#include "include/events.h"
#include "include/wifi_handler.h"
#include "include/program.h"

// Garage MAC is e8:6b:ea:fb:ef:54
// Remote MAC is e8:6b:ea:fb:47:e8
// Using 2.4G channel 13

static const uint32_t magic_value = 0x494d4556;

static mive_program_t* program_g = NULL;

static int s_retry_num = 0;

static const char *TAG = "wifi_handler";
static uint8_t s_remote_mac[ESP_NOW_ETH_ALEN] = {0xe8, 0x6b, 0xea, 0xfb, 0x47, 0xe8};

static const uint8_t PMK[] = {0x18,0x15,0x01,0xfb,0x6c,0x71,0x9e,0x14,0x38,0x69,0xd5,0xb9,0x2a,0x1f,0xac,0x7a};
static const uint8_t LMK[] = {0x1b,0x3d,0x02,0xc4,0x92,0x04,0x03,0x72,0xe4,0x3f,0xcf,0xff,0xc2,0xd4,0x59,0xd7};

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
  mive_event_t mive_event = {0};
  uint8_t * mac_addr = recv_info->src_addr;
  uint8_t * des_addr = recv_info->des_addr;
  uint32_t* magic_val = (uint32_t*)data;

  if (mac_addr == NULL || data == NULL || len <= 0) {
    ESP_LOGE(TAG, "Receive cb arg error");
    return;
  }

  if(*magic_val == magic_value)
  { 
    printf("Got magic val\n");
    mive_event.event_type = MIVE_EVENT_START_GARAGE;
    xQueueSend(program_g->main_queue, &mive_event, pdMS_TO_TICKS(10));
  }
  printf("ESPNOW Data from:");
  for(uint8_t i = 0; i < 6; ++i)
  {
    printf(" %02x", mac_addr[i]);
  }
  printf(" = ");
  for(uint8_t i = 0; i < len; ++i)
  {
    printf("%02x ", data[i]);
  }
  printf("\n");

}

static void espnow_start()
{
  ESP_ERROR_CHECK( esp_now_init() );
  ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
  /* Set primary master key. */
  ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)PMK) );

  /* Add broadcast peer information to peer list. */
  esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
  memset(peer, 0, sizeof(esp_now_peer_info_t));
  peer->channel = 13;
  peer->ifidx = ESP_IF_WIFI_STA;
  peer->encrypt = true;
  memcpy(peer->lmk, LMK, 16);
  memcpy(peer->peer_addr, s_remote_mac, ESP_NOW_ETH_ALEN);
  ESP_ERROR_CHECK( esp_now_add_peer(peer) );
  free(peer);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
  mive_program_t* program = (mive_program_t*)arg;
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_mqtt_client_disconnect(program->mqtt_client);
    if (s_retry_num < 10) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      esp_now_deinit();
      esp_wifi_stop();
      esp_wifi_start();
      espnow_start();
      s_retry_num = 0;
    }
    ESP_LOGI(TAG,"connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    esp_mqtt_client_reconnect(program->mqtt_client);
    esp_netif_sntp_start();
  }
}

void wifi_init_sta(mive_program_t* program)
{
  program_g = program;
  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      program,
                                                      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &wifi_event_handler,
                                                      program,
                                                      &instance_got_ip));

  wifi_config_t wifi_config = {
    .sta = {
      .ssid = WIFI_SSID,
      .password = WIFI_PWD,
      /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
        * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
        * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
        * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
        */
      .threshold.authmode = WIFI_AUTH_WPA2_PSK
    },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );
  espnow_start();
  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
  config.start = false;
  esp_netif_sntp_init(&config);
  ESP_LOGI(TAG, "wifi_init_sta finished.");
}
