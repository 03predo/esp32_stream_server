#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"

#ifndef WIFI_H
#define WIFI_H

// this file should be create by user to define WIFI_SSID and WIFI_PASS
#include "esp32_config/wifi_config.h"

#ifndef WIFI_SSID
#error "WIFI_SSID not defined in esp32_config/wifi_config.h"
#endif 

#ifndef WIFI_PASS
#error "WIFI_PASS not defined in esp32_config/wifi_config.h"
#endif 

#define MAXIMUM_RETRY       5
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

void wifi_init_sta(EventGroupHandle_t s_wifi_event_group, unsigned short int *status, char * ip);
void wifi_event_handler(void* s_wifi_event_group, esp_event_base_t event_base, int32_t event_id, void* event_data);

#endif
