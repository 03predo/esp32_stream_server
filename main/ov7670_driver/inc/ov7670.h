#include <stdio.h>
#include <stdint.h>


#define BYTES_PER_PIXEL 2
#define IMAGE_WIDTH_PIXELS 320
#define IMAGE_HEIGHT_PIXELS 240
#define LINE_WIDTH_BYTES (IMAGE_WIDTH_PIXELS * BYTES_PER_PIXEL)
#define IMAGE_SIZE_BYTES (LINE_WIDTH_BYTES * IMAGE_HEIGHT_PIXELS)

typedef struct {
    uint8_t* buf;
    size_t size;
} ov7670_frame_t;

esp_err_t ov7670_init();
esp_err_t ov7670_capture(ov7670_frame_t* img);
esp_err_t ov7670_release(ov7670_frame_t* img);
