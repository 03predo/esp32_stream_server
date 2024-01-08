#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ov7670_ll.h"
#include "ov7670_pin_defs.h"
#include "ov7670_regs.h"
#include "sccb.h"
#include "xclk.h"

#define OV7670_SCCB_ADDR 0x21
#define MAX_RETRY 10

static ov7670_t ov7670;

static const char* TAG = "ov7670_hal";

static int ov7670_write_array(struct regval_list *vals){
    int ret = 0;
	
	while ( (vals->reg_num != 0xff || vals->value != 0xff) && (ret == 0) ) {
        ret = SCCB_Write(OV7670_SCCB_ADDR, vals->reg_num, vals->value);

	    ESP_LOGD(TAG, "reset reg %02X, W(%02X) R(%02X)", vals->reg_num, 
                        vals->value, SCCB_Read(OV7670_SCCB_ADDR, vals->reg_num) );
		
		vals++;
	}

    return ret;
}

static int ov7670_reset()
{
    int ret;

    // Reset all registers
    SCCB_Write(OV7670_SCCB_ADDR, COM7, COM7_RESET);

    // Delay 10 ms
    vTaskDelay(10 / portTICK_PERIOD_MS);

    ret = ov7670_write_array(ov7670_default_regs);
    // Delay
    vTaskDelay(30 / portTICK_PERIOD_MS);

    return ret;
}


static int ov7670_frame_control(int hstart, int hstop, int vstart, int vstop)
{
    struct regval_list frame[7];

    frame[0].reg_num = HSTART;
    frame[0].value = (hstart >> 3);

    frame[1].reg_num = HSTOP;
    frame[1].value = (hstop >> 3);

    frame[2].reg_num = HREF;
    frame[2].value = (((hstop & 0x07) << 3) | (hstart & 0x07));
    
    frame[3].reg_num = VSTART;
    frame[3].value = (vstart >> 2);
    
    frame[4].reg_num = VSTOP;
    frame[4].value = (vstop >> 2);

    frame[5].reg_num = VREF;
    frame[5].value = (((vstop & 0x02) << 2) | (vstart & 0x02));

    /* End mark */
    frame[6].reg_num = 0xFF;
    frame[6].value = 0xFF;

    return ov7670_write_array(frame);
}

static int ov7670_config()
{
    int ov7670_clkrc =  SCCB_Read(OV7670_SCCB_ADDR, CLKRC);
    
    if(ov7670_write_array(ov7670_qvga) != ESP_OK){
        ESP_LOGI(TAG, "failed to set frame size to qvga");
        return ESP_FAIL;

    }
    ov7670_frame_control(158, 14, 10, 490);
    vTaskDelay(30 / portTICK_PERIOD_MS);

    if(ov7670_write_array(ov7670_fmt_rgb565) != ESP_OK){
        ESP_LOGI(TAG, "failed to set pix format to rgb565");
        return ESP_FAIL;
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);
    
    SCCB_Write(OV7670_SCCB_ADDR, CLKRC, ov7670_clkrc); 
    return ESP_OK;
}



static int set_colorbar(int enable)
{
    uint8_t ret = 0;
    // Read register scaling_xsc
    uint8_t reg = SCCB_Read(OV7670_SCCB_ADDR, SCALING_XSC);

    // Pattern to set color bar bit[0]=0 in every case
    reg = SCALING_XSC_CBAR(reg);

    // Write pattern to SCALING_XSC
    ret = SCCB_Write(OV7670_SCCB_ADDR, SCALING_XSC, reg);

    // Read register scaling_ysc
    reg = SCCB_Read(OV7670_SCCB_ADDR, SCALING_YSC);

    // Pattern to set color bar bit[0]=0 in every case
    reg = SCALING_YSC_CBAR(reg, enable);

    // Write pattern to SCALING_YSC
    ret = ret | SCCB_Write(OV7670_SCCB_ADDR, SCALING_YSC, reg);

    // return 0 or 0xFF
    return ret;
}

esp_err_t ov7670_capture(ov7670_frame_t* img){
    if(ov7670.frame_in_use){
        ESP_LOGE(TAG, "frame already in use");
        return ESP_FAIL;
    }
    size_t buf_cnt = 0; // update with correct name and type
    size_t retry_num = 0;

    ov7670_event_t event;
    ov7670.state = OV7670_IDLE;
    xQueueReset(ov7670.event_queue);
    ll_vsync_intr_enable(1);
    BaseType_t ret = xQueueReceive(ov7670.event_queue, &event, portMAX_DELAY);
    if(ret != pdPASS || event != VSYNC_EVENT){
        ESP_LOGE(TAG, "failed to receive vsync event");
        ll_vsync_intr_enable(0);
        return ESP_FAIL;
    }
    ll_i2s_start(&ov7670);
    buf_cnt = 0;
    ov7670.frame.size = 0;

    while(1){ 
        BaseType_t ret = xQueueReceive(ov7670.event_queue, &event, portMAX_DELAY);
        if(ret != pdPASS){
            ESP_LOGE(TAG, "failed to receive event from queue");
            ll_i2s_stop(&ov7670);
            ll_vsync_intr_enable(0);
            return ESP_FAIL;
        }
        switch(event){
            case IN_SUC_EOF_EVENT:
                if((ov7670.frame.size + DMA_HALF_BUF_SIZE_BYTES / 2) > IMAGE_SIZE_BYTES){
                    ESP_LOGE(TAG, "frame size exceeds image size: %u > %u", (ov7670.frame.size + DMA_HALF_BUF_SIZE_BYTES / 2), IMAGE_SIZE_BYTES);
                    ll_i2s_stop(&ov7670);
                    ll_vsync_intr_enable(0);
                    return ESP_FAIL;
                }
                ov7670.frame.size += ll_memcpy(&ov7670.frame.buf[ov7670.frame.size],
                                                    &ov7670.dma_buf[(buf_cnt % 2) * DMA_HALF_BUF_SIZE_BYTES],
                                                    DMA_HALF_BUF_SIZE_BYTES);
                buf_cnt++;
                break;
            case VSYNC_EVENT:
                ll_i2s_stop(&ov7670);
                ll_vsync_intr_enable(0);
                if(ov7670.frame.size != IMAGE_SIZE_BYTES){
                    ESP_LOGE(TAG, "frame size doesn't equal image size: %u != %u", ov7670.frame.size, IMAGE_SIZE_BYTES);
                    return ESP_FAIL;
                }
                ESP_LOGI(TAG, "successful capture");
                img->buf = ov7670.frame.buf;
                img->size = ov7670.frame.size;
                ov7670.frame_in_use = true;
                return ESP_OK;
        }
    }
}

esp_err_t ov7670_release(ov7670_frame_t* img){
    img->buf = NULL;
    img->size = 0;
    ov7670.frame_in_use = false;
    return ESP_OK;
}


esp_err_t ov7670_init(){
    ll_pin_config(&ov7670);
    ll_i2s_config();
    camera_enable_out_clock(LEDC_TIMER_0, LEDC_CHANNEL_0, OV7670_PIN_XCLK, 10000000);
    SCCB_Init(OV7670_PIN_SIOD, OV7670_PIN_SIOC);
    if(SCCB_Probe_Addr(OV7670_SCCB_ADDR, 5) != ESP_OK){
       ESP_LOGI(TAG, "failed to probe ov7670"); 
       return ESP_FAIL;
    }
    if(ov7670_reset() != ESP_OK){
        ESP_LOGI(TAG, "failed to reset ov7670");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "successfully probed ov7670");
 
    if(ov7670_config() != ESP_OK){
        ESP_LOGI(TAG, "failed to config ov7670");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "successfully configured ov7670");

    //set_colorbar(1);
    
    ov7670.dma_buf = (uint8_t*)heap_caps_malloc(2 * DMA_HALF_BUF_SIZE_BYTES * sizeof(uint8_t), MALLOC_CAP_DMA);
    if(ov7670.dma_buf == NULL){
        ESP_LOGI(TAG, "failed to alloc dma_buf");
        heap_caps_free(ov7670.dma_buf);
        return ESP_FAIL;
    }

    ov7670.frame.buf = (uint8_t*)heap_caps_malloc(IMAGE_SIZE_BYTES * sizeof(uint8_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if(ov7670.frame.buf == NULL) {
        ESP_LOGI(TAG, "failed to alloc frame buf, size=%ld", IMAGE_SIZE_BYTES);
        heap_caps_free(ov7670.frame.buf);
        return ESP_FAIL;
    }
    ov7670.frame_in_use = false;
    ov7670.dma = (lldesc_t *)heap_caps_malloc(DMA_NODE_CNT * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if(ov7670.dma == NULL) {
        ESP_LOGI(TAG, "failed to alloc dma");
        heap_caps_free(ov7670.dma);
        return ESP_FAIL;
    }

    for(int x = 0; x < DMA_NODE_CNT; x++) {
        ov7670.dma[x].size = NODE_BUF_SIZE_BYTES;
        ov7670.dma[x].length = 0;
        ov7670.dma[x].sosf = 0;
        ov7670.dma[x].eof = 0;
        ov7670.dma[x].owner = 1;
        ov7670.dma[x].buf = (ov7670.dma_buf + NODE_BUF_SIZE_BYTES * x);
        ov7670.dma[x].empty = (uint32_t)&ov7670.dma[(x + 1) % DMA_NODE_CNT];
    }

    ESP_LOGI(TAG, "DMA_HALF_BUF_SIZE_BYTES=%u, DMA_NODE_CNT=%u", DMA_HALF_BUF_SIZE_BYTES, DMA_NODE_CNT);

    ov7670.event_queue = xQueueCreate(1, sizeof(ov7670_event_t));
    if(ov7670.event_queue == NULL){
        return ESP_FAIL;
    }
    ll_intr_alloc(&ov7670); 

    return ESP_OK;
}

esp_err_t ov7670_destroy(){
    heap_caps_free(ov7670.dma_buf);
    heap_caps_free(ov7670.frame.buf);
    heap_caps_free(ov7670.dma);
    return ESP_OK;
}
