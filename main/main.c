#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"

#include "wifi.h"
#include "ov7670.h"


#define PORT 2848

#define IMAGE_SEGMENT_NUM 4
#define IMAGE_SEGMENT_SIZE (IMAGE_SIZE_BYTES / IMAGE_SEGMENT_NUM)

#define START       0x0
#define DATA        0x1
#define RETRANSMIT  0x2
#define END         0x3

static const char* TAG = "main";

uint8_t msg[IMAGE_SEGMENT_SIZE + 2];

EventGroupHandle_t wifi_event_group;

void app_main(void)
{
    if(ov7670_init() != ESP_OK){
        ESP_LOGE(TAG, "failed to init");
        return;
    }  

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    unsigned short int status = 0;
    char branch_ip[50];
    wifi_init_sta(wifi_event_group, &status, branch_ip);
    ESP_LOGI(TAG, "ip=%s", branch_ip);

    // create socket
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    // bind socket
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
        .sin_addr.s_addr = inet_addr(branch_ip),
    };
    int ret = bind(fd, (struct sockaddr*) &server_addr, sizeof(server_addr));
    if(ret < 0){
        ESP_LOGI(TAG, "bind failed");
        return;
    }

    struct sockaddr_in client_addr;
    uint32_t client_struct_length = sizeof(client_addr); 

    ov7670_frame_t frame;
    while(1){
        ESP_LOGD(TAG, "waiting for message from client");
        uint8_t client_msg[2];
        int bytes_received = recvfrom(fd, client_msg, sizeof(client_msg), 0, (struct sockaddr*) &client_addr, &client_struct_length);
        if(bytes_received == -1){
            ESP_LOGI(TAG, "recv failed(%u)", errno);
            continue;
        }

        if(client_msg[0] == START){
            ESP_LOGD(TAG, "received START from client, capturing new frame");
            if(ov7670_release(&frame) != ESP_OK){
                ESP_LOGE(TAG, "failed release");
            }
            while(ov7670_capture(&frame) != ESP_OK){
                ov7670_release(&frame);
                ESP_LOGW(TAG, "failed capture");
            }
        }

        ESP_LOGD(TAG, "client seq_num request: %#x", client_msg[1]);
        for(uint8_t i = 0; i < IMAGE_SEGMENT_NUM; ++i){
            if(!(client_msg[1] & (1 << i))){
                continue;
            }
            memcpy(msg + 2, frame.buf + IMAGE_SEGMENT_SIZE * i, IMAGE_SEGMENT_SIZE);
            msg[0] = DATA;
            msg[1] = i;
            int tmp = sendto(fd, msg, IMAGE_SEGMENT_SIZE + 2, 0, (struct sockaddr*) &client_addr, client_struct_length);
            if(tmp == -1){
                if(errno != ENOMEM){
                    ESP_LOGD(TAG, "send failed(%d)\n", errno);
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                    break;
                }
                ESP_LOGW(TAG, "ENOMEM occurred");
                continue;
            }
            ESP_LOGD(TAG, "sent seq_num: %u, size: %d bytes", i, tmp);
        }

        uint8_t end_msg[2];
        end_msg[0] = END;
        sendto(fd, end_msg, sizeof(end_msg), 0, (struct sockaddr*) &client_addr, client_struct_length);      
    } 
    return;
}
