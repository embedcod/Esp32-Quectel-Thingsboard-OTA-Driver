/*
 * gsm_ota.h
 *
 *  Created:  30/04/2026
 *  Modified: 21/05/2026
 *
 *  Author: ERIC MULWA
 *
 *  Description:
 *      Native ESP-IDF OTA driver for Quectel modems.
 *      Integrates with ThingsBoard IoT platform for firmware attribute
 *      retrieval, streaming, verification, and telemetry reporting.
 *      Designed for production field deployment — robust, fail-safe,
 *      and rollback-aware.
 */

#ifndef QUECTEL_H
#define QUECTEL_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include <stdio.h>
#include "string.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "nvs.h"
#include "cJSON.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_vfs_fat.h"
#include "cJSON.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <bootloader_common.h>
#include "esp_app_desc.h"
#include <sdkconfig.h>
#include "esp_partition.h"

// Definitions
#define MODEM_UART_NUM UART_NUM_1
#define TX_PIN 17
#define RX_PIN 16
#define RESET_PIN 19
#define PWR_PIN 18 
#define GSM_EN 4
#define ESP_BAUDRATE 115200
#define QUECTEL_BAUDRATE 115200
#define TAG_GSM "OTA"
#define BUF_SIZE 8192
#define APN "safaricomiot"

/*******************************************************************************************************************************
FIRMWARE: METADATA
*******************/ 
#define FIRMWARE_TITLE    "gsm_ota"
#define FIRMWARE_VERSION  "v1.21"

typedef enum {
    ESP_HTTPS_OTA_INIT,
    ESP_HTTPS_OTA_BEGIN,
    ESP_HTTPS_OTA_IN_PROGRESS,
    ESP_HTTPS_OTA_SUCCESS,
} esp_https_ota_state;

/**
 * GSM OTA Update using esp_https_ota-like interface
 */
typedef struct {
    esp_ota_handle_t update_handle;
    const esp_partition_t *update_partition;
    int file_size;
    int bytes_received;
    uint8_t *ota_buffer;
    int buffer_size;
    bool bulk_flash_erase;
    esp_https_ota_state state;
} gsm_ota_handle_t;


static const int RETRIES = 3;
static const int CMD_DELAY_MS = 1000;

/**
 * @brief Gsm function prototypes
 */ 
void uart_init();
void initialize_nvs();
void gsm_reset();
void gsm_poweron();
void hardware_poweroff();
bool send_at_command(const char *command, const char *expected_response, int retries, uint32_t timeout_ms);
bool poweron_modem();
bool activate_pdp();
bool deactivate_pdp();
void powerdown_modem();

/**
 * @brief Ota function prototypes
 */ 
uint32_t otagetChipId();
void otagetChipIdString(char *device_token, size_t size);
esp_err_t gsm_ota_abort(gsm_ota_handle_t *handle);
esp_err_t gsm_ota_finish(gsm_ota_handle_t *handle);
int gsm_ota_get_bytes_received(gsm_ota_handle_t *handle);
bool gsm_ota_is_complete_data_received(gsm_ota_handle_t *handle);
esp_err_t gsm_ota_perform(gsm_ota_handle_t *handle);
esp_err_t gsm_ota_write_data(gsm_ota_handle_t *handle, const void *data, size_t len);
esp_err_t gsm_ota_verify_chip_id(const void *data);
int gsm_ota_read_data(gsm_ota_handle_t *handle, uint8_t *buffer, int max_len, int timeout_ms);
esp_err_t gsm_ota_begin(gsm_ota_handle_t *handle, int firmware_size, bool bulk_erase);
bool get_thingsboard_fw_info(void);
bool perform_secure_ota_update();
bool tb_report_pre_stream_status(void);
bool tb_report_post_stream_status(int ota);
bool perform_ota();
bool is_ota_required(void);
void running_partition();

#endif // QUECTEL_H
