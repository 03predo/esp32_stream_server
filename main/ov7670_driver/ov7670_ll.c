#include <stdio.h>
#include <stdint.h>

#include "soc/i2s_struct.h"
#include "esp_err.h"
#include "esp_private/periph_ctrl.h"
#include "hal/gpio_ll.h"
#include "esp_log.h"
#include "esp32/rom/ets_sys.h"
#include "driver/gpio.h"

#include "rom/gpio.h"

#include "ov7670_ll.h"
#include "ov7670_pin_defs.h"

#define I2S_ISR_ENABLE(i) {I2S0.int_clr.i = 1;I2S0.int_ena.i = 1;}
#define I2S_ISR_DISABLE(i) {I2S0.int_ena.i = 0;I2S0.int_clr.i = 1;}

typedef union {
    struct {
        uint32_t sample2:8;
        uint32_t unused2:8;
        uint32_t sample1:8;
        uint32_t unused1:8;
    };
    uint32_t val;
} dma_elem_t;

static const char * TAG = "ov7670_ll";

bool IRAM_ATTR ll_i2s_stop()
{
    I2S0.conf.rx_start = 0;
    I2S0.int_ena.in_suc_eof = 0;
    I2S0.int_clr.in_suc_eof = 1; 
    I2S0.in_link.stop = 1;
    return true;
}

void IRAM_ATTR ll_send_event(ov7670_t* ov7670, ov7670_event_t event, BaseType_t * HPTaskAwoken)
{
    if (xQueueSendFromISR(ov7670->event_queue, (void *)&event, HPTaskAwoken) != pdTRUE) {
        ll_i2s_stop();
        ov7670->state = OV7670_IDLE;
        ets_printf(DRAM_STR("ov7670: EV-%s-OVF\r\n"), event==IN_SUC_EOF_EVENT ? DRAM_STR("EOF") : DRAM_STR("VSYNC"));
    }
}

static void IRAM_ATTR ll_vsync_isr(void *arg)
{
    ov7670_t* ov7670 = (ov7670_t*) arg;
    BaseType_t HPTaskAwoken = pdFALSE;

    ets_delay_us(1);
    if (gpio_ll_get_level(&GPIO, OV7670_PIN_VSYNC) == 0) {
        ll_send_event(ov7670, VSYNC_EVENT, &HPTaskAwoken);
        if (HPTaskAwoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

void ll_vsync_intr_enable(bool en)
{
    if (en) {
        gpio_intr_enable(OV7670_PIN_VSYNC);
    } else {
        gpio_intr_disable(OV7670_PIN_VSYNC);
    }
}

static void IRAM_ATTR ll_dma_isr(void *arg)
{
    ov7670_t* ov7670 = (ov7670_t*)arg;
    BaseType_t HPTaskAwoken = pdFALSE;

    typeof(I2S0.int_st) status = I2S0.int_st;
    if (status.val == 0) {
        return;
    }

    I2S0.int_clr.val = status.val;

    if (status.in_suc_eof) {
        ll_send_event(ov7670, IN_SUC_EOF_EVENT, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}


esp_err_t ll_pin_config(ov7670_t* ov7670) {
    ESP_LOGI(TAG, "configuring pins");
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = 1ULL << OV7670_PIN_VSYNC;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(OV7670_PIN_VSYNC, ll_vsync_isr, ov7670);
    gpio_intr_disable(OV7670_PIN_VSYNC);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[OV7670_PIN_PCLK], PIN_FUNC_GPIO);
    gpio_set_direction(OV7670_PIN_PCLK, GPIO_MODE_INPUT);
    gpio_set_pull_mode(OV7670_PIN_PCLK, GPIO_FLOATING);
    gpio_matrix_in(OV7670_PIN_PCLK, I2S0I_WS_IN_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[OV7670_PIN_VSYNC], PIN_FUNC_GPIO);
    gpio_set_direction(OV7670_PIN_VSYNC, GPIO_MODE_INPUT);
    gpio_set_pull_mode(OV7670_PIN_VSYNC, GPIO_FLOATING);
    gpio_matrix_in(OV7670_PIN_VSYNC, I2S0I_V_SYNC_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[OV7670_PIN_HREF], PIN_FUNC_GPIO);
    gpio_set_direction(OV7670_PIN_HREF, GPIO_MODE_INPUT);
    gpio_set_pull_mode(OV7670_PIN_HREF, GPIO_FLOATING);
    gpio_matrix_in(OV7670_PIN_HREF, I2S0I_H_SYNC_IDX, false);

    int data_pins[8] = {
        OV7670_PIN_D0, OV7670_PIN_D1, OV7670_PIN_D2, OV7670_PIN_D3, OV7670_PIN_D4, OV7670_PIN_D5, OV7670_PIN_D6, OV7670_PIN_D7,
    };
    for (int i = 0; i < 8; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[data_pins[i]], PIN_FUNC_GPIO);
        gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
        gpio_matrix_in(data_pins[i], I2S0I_DATA_IN0_IDX + i, false);
    }

    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);

    return ESP_OK;
}

bool ll_i2s_start(ov7670_t *ov7670)
{
    I2S0.conf.rx_start = 0;

    I2S_ISR_ENABLE(in_suc_eof);

    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;

    // RX_EOF_NUM configures size of data for single transfer in words
    // this is the amount of data received on an IN_SUC_EOF interrupt
    I2S0.rx_eof_num = DMA_HALF_BUF_SIZE_BYTES / sizeof(dma_elem_t);
    I2S0.in_link.addr = ((uint32_t)&ov7670->dma[0]) & 0xfffff;

    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;
    return true;
}

esp_err_t ll_i2s_config() {
    ESP_LOGI(TAG, "configuring i2s peripheral");
    periph_module_enable(PERIPH_I2S0_MODULE);

    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;

    I2S0.conf.rx_slave_mod = 1;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;

    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;

    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 2;

    I2S0.fifo_conf.dscr_en = 1;
    I2S0.fifo_conf.rx_fifo_mod = 1; // 16-bit single channel
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;

    I2S0.conf_chan.rx_chan_mod = 1;
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.timing.val = 0;
    I2S0.timing.rx_dsync_sw = 1;

    return ESP_OK;
}

size_t IRAM_ATTR ll_memcpy(uint8_t* dst, const uint8_t* src, size_t len)
{
    const dma_elem_t* dma_el = (const dma_elem_t*)src;
    size_t elements = len / sizeof(dma_elem_t);
    size_t end = elements / 4;
    for (size_t i = 0; i < end; ++i) {
        dst[0] = dma_el[0].sample1;//y0
        dst[1] = dma_el[0].sample2;//u
        dst[2] = dma_el[1].sample1;//y1
        dst[3] = dma_el[1].sample2;//v

        dst[4] = dma_el[2].sample1;//y0
        dst[5] = dma_el[2].sample2;//u
        dst[6] = dma_el[3].sample1;//y1
        dst[7] = dma_el[3].sample2;//v
        dma_el += 4;
        dst += 8;
    }
    return elements * 2;
}

esp_err_t ll_intr_alloc(ov7670_t* ov7670){
    return esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, ll_dma_isr, ov7670, &ov7670->intr_handle);
}

