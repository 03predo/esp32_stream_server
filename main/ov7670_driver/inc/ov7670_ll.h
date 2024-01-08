#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp32/rom/lldesc.h"

#include "ov7670.h"

#define BYTES_PER_PIXEL 2
#define LINE_WIDTH_BYTES (IMAGE_WIDTH_PIXELS * BYTES_PER_PIXEL)

#define DMA_BUF_SIZE_MAX_BYTES (1 << 15U)
#define NODE_BUF_SIZE_MAX_BYTES (1 << 12U)

#define DMA_HALF_BUF_MAX_PIXELS ((DMA_BUF_SIZE_MAX_BYTES / 2) / BYTES_PER_PIXEL)
#define NODE_MAX_PIXELS (NODE_BUF_SIZE_MAX_BYTES / BYTES_PER_PIXEL)

#if (NODE_BUF_SIZE_MAX_BYTES >= LINE_WIDTH_BYTES)
#define LINES_PER_NODE (NODE_BUF_SIZE_MAX_BYTES / LINE_WIDTH_BYTES)
#else
#error max node size is less then line width
#endif

#define NODE_BUF_SIZE_BYTES (LINES_PER_NODE * LINE_WIDTH_BYTES)
#define DMA_HALF_BUF_SIZE_BYTES (((DMA_BUF_SIZE_MAX_BYTES / 2) / NODE_BUF_SIZE_BYTES) * NODE_BUF_SIZE_BYTES)
#define DMA_NODE_CNT ((DMA_HALF_BUF_SIZE_BYTES * 2) / NODE_BUF_SIZE_BYTES) // number of nodes in a dma buffer

typedef enum {
    IN_SUC_EOF_EVENT = 0,
    VSYNC_EVENT
}ov7670_event_t;

typedef enum {
    OV7670_IDLE = 0,
    OV7670_READ
}ov7670_state_t;

typedef struct {
    QueueHandle_t event_queue;
    TaskHandle_t task_handle;
    ov7670_state_t state;
    ov7670_frame_t frame;
    bool frame_in_use;
    lldesc_t *dma;
    uint8_t* dma_buf;
    intr_handle_t intr_handle;
}ov7670_t;


esp_err_t ll_pin_config(ov7670_t* ov7670);
esp_err_t ll_i2s_config();
esp_err_t ll_intr_alloc(ov7670_t* ov7670);
bool ll_i2s_start(ov7670_t *ov7670);
bool IRAM_ATTR ll_i2s_stop();
void ll_vsync_intr_enable(bool en);
size_t IRAM_ATTR ll_memcpy(uint8_t* dst, const uint8_t* src, size_t len);
