#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>
#include <time.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "string.h"
#include <time.h>
#include "nvs.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "driver/rmt.h"
#include "gsm_ota.h"

/*******************************************************************************************************************************
DEFINITIONS: GLOBAL
*******************/ 
#define LED_PIN 15
#define RUN_TIME_BEFORE_OTA_MS 60000  // 60 second (s)
/*******************************************************************************************************************************/

/*******************************************************************************************************************************
GLOBAL: Global Definitions
**************************/     
static const char *TAG = "OTA";
/*******************************************************************************************************************************/


/*******************************************************************************************************************************
INIT: WS2812 LED
********************************/
#define NUM_LEDS 1
#define LED_RMT_TX_CHANNEL RMT_CHANNEL_0
#define LED_RMT_TX_GPIO 15
#define BITS_PER_LED_CMD 24
#define LED_BUFFER_ITEMS (NUM_LEDS * BITS_PER_LED_CMD)
#define T0H 14   
#define T1H 52   
#define T0L 52   
#define T1L 14  

static rmt_item32_t led_data_buffer[LED_BUFFER_ITEMS];
static uint8_t brightness = 200;  // Brightness (0 - 255)

/*******************************************************************************************************************************/



/*********************************************************************************************************************************
FUNCTIONS: PROTOTYPES
**********************/
// WS2812 LED FUNCTIONS
static uint32_t apply_brightness(uint32_t color);
static void setup_rmt_data_buffer(uint32_t leds[NUM_LEDS]);
static esp_err_t ws2812_control_init(void);
static esp_err_t ws2812_write_leds(uint32_t leds[NUM_LEDS]);
void blink_led();
void error(int count);
/*******************************************************************************************************************************
FUNCTION: THE MAIN APPLICATION
*****************************/
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000));
	ESP_LOGI(TAG, "Secure GSM OTA Firmware started");
	vTaskDelay(pdMS_TO_TICKS(2000));
	ws2812_control_init();
	vTaskDelay(pdMS_TO_TICKS(1000));
	initialize_nvs();
	vTaskDelay(pdMS_TO_TICKS(1000));
    
    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
   
    while (1) {
        blink_led();
        if ((xTaskGetTickCount() * portTICK_PERIOD_MS - start_time) >= RUN_TIME_BEFORE_OTA_MS) {
            ESP_LOGI(TAG, "GSM OTA triggered, starting OTA update...");
            // Gsm ota execution (Requires a buffer size of 8192, allocate calling task accordingly)			
			if (!perform_ota()) {
				error(3);
			    ESP_LOGE(TAG_GSM, "Ota update execution failed!");
			    // Take action (optional)
			    vTaskDelay(pdMS_TO_TICKS(1000));
			    continue;
		    } else { /* on success device restarts.*/}
		 
            start_time = xTaskGetTickCount() * portTICK_PERIOD_MS; // reset timer
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// WS2812 FUNCTIONS

/**
 * @brief Brightness Control
 */
static uint32_t apply_brightness(uint32_t color) {
    uint8_t r = ((color >> 16) & 0xFF) * brightness / 255;
    uint8_t g = ((color >> 8) & 0xFF) * brightness / 255;
    uint8_t b = (color & 0xFF) * brightness / 255;
    return (r << 16) | (g << 8) | b;
}

/**
 * @brief  RMT Buffer
 */
static void setup_rmt_data_buffer(uint32_t leds[NUM_LEDS]) {
    for (uint32_t led = 0; led < NUM_LEDS; led++) {
        uint32_t bits_to_send = apply_brightness(leds[led]);
        uint32_t mask = 1 << (BITS_PER_LED_CMD - 1);

        for (uint32_t bit = 0; bit < BITS_PER_LED_CMD; bit++) {
            uint32_t bit_is_set = bits_to_send & mask;
            led_data_buffer[(led * BITS_PER_LED_CMD) + bit] = bit_is_set ?
                (rmt_item32_t){{{T1H, 1, T1L, 0}}} :
                (rmt_item32_t){{{T0H, 1, T0L, 0}}};
            mask >>= 1;
        }
    }
}

/**
 * @brief Initialize WS2812
 */
static esp_err_t ws2812_control_init(void) {
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = LED_RMT_TX_CHANNEL,
        .gpio_num = LED_RMT_TX_GPIO,
        .mem_block_num = 3,
        .tx_config.loop_en = false,
        .tx_config.carrier_en = false,
        .tx_config.idle_output_en = true,
        .tx_config.idle_level = 0,
        .clk_div = 2
    };
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    return ESP_OK;
}

/**
 * @brief Write to the LEDs
 */
static esp_err_t ws2812_write_leds(uint32_t leds[NUM_LEDS]) {
    setup_rmt_data_buffer(leds);
    ESP_ERROR_CHECK(rmt_write_items(LED_RMT_TX_CHANNEL, led_data_buffer, LED_BUFFER_ITEMS, false));
    ESP_ERROR_CHECK(rmt_wait_tx_done(LED_RMT_TX_CHANNEL, portMAX_DELAY));
    return ESP_OK;
}

// LED blinking patterns
void blink_led() {
    uint32_t led_colors[NUM_LEDS];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < NUM_LEDS; j++) {
            led_colors[j] = apply_brightness(0x000000); 
        }
        ws2812_write_leds(led_colors);
        vTaskDelay(pdMS_TO_TICKS(500)); 
        for (int i = 0; i < NUM_LEDS; i++) {
            led_colors[i] = apply_brightness(0xFF0000); 
        }
        ws2812_write_leds(led_colors);
        vTaskDelay(pdMS_TO_TICKS(500)); 
    }
    for (int j = 0; j < NUM_LEDS; j++) {
        led_colors[j] = apply_brightness(0x000000); 
    }
    ws2812_write_leds(led_colors);   
}

 /**
 * @brief error indicator. 
 */
 void error(int count) {
    uint32_t led_colors[NUM_LEDS];
    for (int i = 0; i < count; i++) {
        for (int j = 0; j < NUM_LEDS; j++) {
            led_colors[j] = apply_brightness(0x000000); 
        }
        ws2812_write_leds(led_colors);
        vTaskDelay(pdMS_TO_TICKS(100)); 
        for (int i = 0; i < NUM_LEDS; i++) {
            led_colors[i] = apply_brightness(0x00FF00); 
        }
        ws2812_write_leds(led_colors);
        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
    for (int j = 0; j < NUM_LEDS; j++) {
        led_colors[j] = apply_brightness(0x000000); 
    }
    ws2812_write_leds(led_colors);
}