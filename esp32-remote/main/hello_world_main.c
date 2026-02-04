/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "iot_button.h"
#include "button_gpio.h"

// Garage MAC is e8:6b:ea:fb:ef:54
// Remote MAC is e8:6b:ea:fb:47:e8
// Using 2.4G channel 13

#define ESPNOW_MAXDELAY 512
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

static const uint32_t magic_value = 0x494d4556;

static QueueHandle_t s_example_espnow_queue = NULL;

static const char *TAG = "esp_remote_main";

static const uint8_t PMK[] = {0x18,0x15,0x01,0xfb,0x6c,0x71,0x9e,0x14,0x38,0x69,0xd5,0xb9,0x2a,0x1f,0xac,0x7a};
static const uint8_t LMK[] = {0x1b,0x3d,0x02,0xc4,0x92,0x04,0x03,0x72,0xe4,0x3f,0xcf,0xff,0xc2,0xd4,0x59,0xd7};

static uint8_t s_example_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static uint8_t s_garage_mac[ESP_NOW_ETH_ALEN] = {0xe8, 0x6b, 0xea, 0xfb, 0xef, 0x54};

struct remote_data {
    uint32_t data;
};

typedef uint32_t example_espnow_event_t;

static void example_espnow_task(void *pvParameter);

static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(13, WIFI_SECOND_CHAN_NONE));
}

static void example_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    uint8_t * mac_addr = recv_info->src_addr;
    uint8_t * des_addr = recv_info->des_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (IS_BROADCAST_ADDR(des_addr)) {
        /* If added a peer with encryption before, the receive packets may be
         * encrypted as peer-to-peer message or unencrypted over the broadcast channel.
         * Users can check the destination address to distinguish it.
         */
        ESP_LOGD(TAG, "Receive broadcast ESPNOW data");
    } else {
        ESP_LOGD(TAG, "Receive unicast ESPNOW data");
    }
}

static esp_err_t example_espnow_init(void)
{
    s_example_espnow_queue = xQueueCreate(20, sizeof(example_espnow_event_t));

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(example_espnow_recv_cb) );
    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vQueueDelete(s_example_espnow_queue);
        s_example_espnow_queue = NULL;
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = 13;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = true;
    memcpy(peer->lmk, LMK, 16);
    memcpy(peer->peer_addr, s_garage_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    xTaskCreate(example_espnow_task, "example_espnow_task", 2048, NULL, 4, NULL);

    return ESP_OK;
}

static void button_event_cb(void *arg, void *data)
{
    button_event_t event = iot_button_get_event(arg);
    ESP_LOGI(TAG, "%s", iot_button_get_event_str(event));
    if (BUTTON_PRESS_REPEAT == event || BUTTON_PRESS_REPEAT_DONE == event) {
        ESP_LOGI(TAG, "\tREPEAT[%d]", iot_button_get_repeat(arg));
    }

    if (BUTTON_PRESS_UP == event || BUTTON_LONG_PRESS_HOLD == event || BUTTON_LONG_PRESS_UP == event) {
        example_espnow_event_t mive_event = 1;
        xQueueSend(s_example_espnow_queue, &mive_event, pdMS_TO_TICKS(50));
        ESP_LOGI(TAG, "\tPressed Time[%"PRIu32"]", iot_button_get_pressed_time(arg));
    }
}


void example_espnow_task(void *pvParameter)
{
    example_espnow_event_t event;
    struct remote_data data = {0};
    button_handle_t btn = NULL;
    const button_config_t btn_cfg = {0};
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = 0,
        .active_level = 0,
    };
    iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn);
    iot_button_register_cb(btn, BUTTON_PRESS_DOWN, NULL, button_event_cb, NULL);
    iot_button_register_cb(btn, BUTTON_PRESS_UP, NULL, button_event_cb, NULL);


    while(1)
    {
        if(xQueueReceive(s_example_espnow_queue, &event, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            data.data = magic_value;
            esp_now_send((uint8_t*)&s_garage_mac, (uint8_t*)&data, sizeof(data));
        }
    }

}

void app_main(void)
{    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    example_wifi_init();
    example_espnow_init();
}
