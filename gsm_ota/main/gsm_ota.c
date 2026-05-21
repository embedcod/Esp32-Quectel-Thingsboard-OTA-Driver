/*
 * gsm_ota.c
 *
 *  Created:  30/04/2026
 *  Modified: 21/05/2026
 *
 *  Author: ERIC MULWA
 *
 *  Description:
 *      Native ESP-IDF OTA driver for Quectel EG-series GSM/LTE modems.
 *      Integrates with ThingsBoard IoT platform for firmware attribute
 *      retrieval, streaming and telemetry reporting.
 *
 *      Architecture overview:
 *
 *        perform_ota()
 *          └─ get_thingsboard_fw_info()      — modem on, GET attrs, modem off
 *          └─ is_ota_required()              — version/title gate
 *          └─ perform_secure_ota_update()
 *                ├─ tb_report_pre_stream_status()  — DOWNLOADING 20→80%
 *                ├─ modem on, HTTPS GET size
 *                ├─ gsm_ota_begin / perform / finish  — stream to flash
 *                ├─ verify_checksum()              — SHA-256 on flash
 *                ├─ URC port restoration (3-stage escalation)
 *                ├─ modem off
 *                └─ tb_report_post_stream_status() — VERIFYING→SUCCESS/FAILED
 *
 * 				  NB: URCs are not prioritized here because they are not used
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_https_ota.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <errno.h>
#include <sys/param.h>
#include <inttypes.h>
#include "gsm_ota.h"

char tb_fw_info[BUF_SIZE];
char response[BUF_SIZE];

/*******************************************************************************************************************************
GLOBAL: Global Definitions
**************************/     
uint32_t storedChipId = 0;
/*******************************************************************************************************************************/

/*******************************************************************************************************************************
THINGSBOARD: GET DEFINITIONS
***************************/  
#define MAX_RESPONSE_SIZE 2048
char http_response_buffer[MAX_RESPONSE_SIZE];
size_t http_response_length = 0;
// ThingsBoard GET credentials
#define THINGSBOARD_HOST "demo.thingsboard.io"
char device_token[32];
// OTA variables
typedef struct {
    char fw_title[128];
    char fw_version[64];
    char fw_checksum[128];
    char fw_checksum_algorithm[32];
} tb_fw_info_t;
tb_fw_info_t tb_info;
// OTA Flags
bool ota_required = false;
bool urc_to_usb = false;
/*******************************************************************************************************************************/

/************************************************************************************************************************************** 
FUNCTIONS: INITIALIZATION
************************/

void initialize_nvs() {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

// Function to get chip ID based on the MAC address
uint32_t otagetChipId() {
    if (storedChipId == 0) {
        uint8_t mac[6];
        esp_base_mac_addr_get(mac); 

        for (int i = 0; i < 6; i++) {
            storedChipId |= ((uint32_t)mac[i] << (8 * i));
        }
    }
    return storedChipId;
}
// Function to get the chip ID as a string
void otagetChipIdString(char *device_token, size_t size) {
    uint32_t chipId = otagetChipId();
    snprintf(device_token, size, "%" PRIu32, chipId);  
    printf("Device Serial Number (Chip ID): %s\n\n", device_token); 
} 

/**
*  @brief UART Configuration
*/
void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = QUECTEL_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(MODEM_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(MODEM_UART_NUM, &uart_config);
    uart_set_pin(MODEM_UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
*  @brief Hardware Reset
*/
void gsm_reset() {
	esp_rom_gpio_pad_select_gpio(RESET_PIN);
    gpio_set_direction(RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(5000));
}

/**
*  @brief Hardware Power On
*/
void gsm_poweron() {
	// connect to power
    esp_rom_gpio_pad_select_gpio(GSM_EN);
	gpio_set_direction(GSM_EN, GPIO_MODE_OUTPUT);
	gpio_set_level(GSM_EN, 1);
	vTaskDelay(pdMS_TO_TICKS(1000));
	// power on with PWR 
	esp_rom_gpio_pad_select_gpio(PWR_PIN);
    gpio_set_direction(PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500)); // minimum of 500ms
    gpio_set_level(PWR_PIN, 1); 
    vTaskDelay(pdMS_TO_TICKS(500)); 
    gpio_set_level(PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(12000)); // Wait for the module to initialize (minimum of 10s)
}

/**
*  @brief Hardware Power Down
*/
void hardware_poweroff() {
	printf("Executing manual power down (PWR)...!\n\n");
	esp_rom_gpio_pad_select_gpio(PWR_PIN);
    gpio_set_direction(PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500)); // minimum of 500ms
    gpio_set_level(PWR_PIN, 1); 
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(PWR_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(500)); 
	// Disconnect power from Quectel   
	gpio_set_level(GSM_EN, 0);
}

/**
*  @brief Send AT Commands with response check
*/
bool send_at_command(const char *command, const char *expected_response, int retries, uint32_t timeout_ms) {

    for (int attempt = 0; attempt < retries; attempt++) {
        uart_write_bytes(MODEM_UART_NUM, command, strlen(command));
        uart_write_bytes(MODEM_UART_NUM, "\r\n", 2);

        uint64_t start_time = esp_timer_get_time();
        int index = 0;

        while ((esp_timer_get_time() - start_time) < (timeout_ms * 1000)) {
            uint8_t data;
            if (uart_read_bytes(MODEM_UART_NUM, &data, 1, 10 / portTICK_PERIOD_MS) > 0) {
                if (index < BUF_SIZE - 1) {
                    response[index++] = (char)data;
                }
            }
        }
        response[index] = '\0';
        // Copy thingsboard firmware info into a separate buffer
        if (strstr(command, "AT+QHTTPREAD=80")) {
			memset(tb_fw_info, 0, BUF_SIZE);
			strcpy(tb_fw_info, response);
        }
        
        ESP_LOGI(TAG_GSM, "Command: %s, Response: %s", command, response);

        if (strstr(response, expected_response)) {
            return true;
        }

        ESP_LOGW(TAG_GSM, "Attempt %d: Command '%s' failed. Retrying...", attempt + 1, command);
        vTaskDelay(pdMS_TO_TICKS(CMD_DELAY_MS));
    }

    ESP_LOGE(TAG_GSM, "Command '%s' failed after %d retries.", command, retries);
    return false;
}

/**
*  @brief Init uart and power on Modem
*/
bool poweron_modem() {
    uart_init();
    vTaskDelay(pdMS_TO_TICKS(1000));
	esp_rom_gpio_pad_select_gpio(PWR_PIN);
    gpio_set_direction(PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PWR_PIN, 0);
	esp_rom_gpio_pad_select_gpio(RESET_PIN);
    gpio_set_direction(RESET_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RESET_PIN, 0);
	gsm_poweron();
 
 	// Quectel GSM Power On Status Check
    bool power = false;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!send_at_command("AT", "OK", 3, 3000)) {
            ESP_LOGW(TAG_GSM, "Modem is Off on check %d. Retrying...", attempt + 1);
            gsm_poweron();
            vTaskDelay(pdMS_TO_TICKS(50 * (attempt + 1)));
            continue;
        } else {
            power = true;
            printf("Modem is On, Proceeding...\n");
            break; 
        }
    }
    if (!power) {
        ESP_LOGE(TAG_GSM, "GSM Power Status Check failed after %d retries.", RETRIES);
        return false;
    } 
    
 	// Detect simcard by checking its readiness
    bool cpin = false;
    for (int attempt = 0; attempt < RETRIES; attempt++) {
        if (!send_at_command("AT+CPIN?", "READY", RETRIES, 3000)) {
            ESP_LOGW(TAG_GSM, "Simcard not ready on check %d. Retrying...", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(50 * (attempt + 1)));
            continue;
        } else {
            cpin = true;
            printf("Simcard ready, Proceeding...\n");
            break; 
        }
    }
    if (!cpin) {
        ESP_LOGE(TAG_GSM, "Simcard Check failed after %d retries.", RETRIES);
        return false;
    }     
    
	// Check if URC are set to the right port (uart1)
	if(urc_to_usb) {
		bool urc_restored = false;
		for (int attempt = 0; attempt < RETRIES; attempt++) {
		    if (!send_at_command("AT+QURCCFG=\"urcport\"", "+QURCCFG: \"urcport\"", RETRIES, CMD_DELAY_MS) ||
	        	!send_at_command("AT+QURCCFG=\"urcport\",\"uart1\"", "OK", RETRIES, CMD_DELAY_MS) ||
	        	!send_at_command("AT+QURCCFG=\"urcport\"", "+QURCCFG: \"urcport\",\"uart1\"", RETRIES, CMD_DELAY_MS)) { 
		        ESP_LOGW(TAG_GSM, "Attempt %d: Failed to query, change, and confirm URC port restoration, retrying...", attempt + 1);
		        vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
		        continue;
		    } else {
		        ESP_LOGI(TAG_GSM, "URC port successfully restored to UART1 on attempt %d.", attempt + 1);
		        urc_restored = true;
		        urc_to_usb = false;
		        break;
		    }
		}	
		if (!urc_restored) {
		    urc_to_usb = true;
		    ESP_LOGE(TAG_GSM, "Critical: Failed to restore URC port to UART1 after all recovery attempts.");
		    return false;
		}		
	}
  return true;	
}

/**
*  @brief Latch to Network, check Signal Strength and Activate PDP
*/
bool activate_pdp() {
	// GSM Network Latch Check
    bool latch = false;
    for (int attempt = 0; attempt < 10; attempt++) {
        if (!send_at_command("ATE0", "OK", RETRIES, CMD_DELAY_MS) ||
        	!send_at_command("AT+QHTTPCFG=\"contextid\",1", "OK", RETRIES, CMD_DELAY_MS) ||
        	!send_at_command("AT+QHTTPCFG=\"responseheader\",0", "OK", RETRIES, CMD_DELAY_MS) || // response header disables (0)
			!send_at_command("AT+CGREG?", "5", RETRIES, CMD_DELAY_MS)) {
            ESP_LOGW(TAG_GSM, "No Network Latch on attempt %d. Retrying...", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1))); // progressive backoff
            continue;
        } else {
            latch = true;
            ESP_LOGI(TAG_GSM, "GSM Network Latch successful!");
            break; 
        }
    }
    if (!latch) {
        ESP_LOGE(TAG_GSM, "GSM Network Latch failed after %d retries. Shutting down and exiting...", RETRIES);
        return false;
    }

	// Check GSM Signal strength before OTA
	bool signal_good = false;
	for (int attempt = 0; attempt < 10; attempt++) {
	    if (!send_at_command("AT+CSQ", "OK", RETRIES, CMD_DELAY_MS)) {
	        ESP_LOGW(TAG_GSM, "Failed to get signal strength on attempt %d. Retrying...", attempt + 1);
	        vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	        continue;
	    } else {	
	        printf("AT+CSQ Response: %s", response);		
	        // Expected response format: +CSQ: <rssi>,<ber>
	        int rssi = -1, ber = -1;	        
	        // Parse the response - look for the CSQ line anywhere in the response
	        char *csq_ptr = strstr(response, "+CSQ:");
	        if (csq_ptr && sscanf(csq_ptr, "+CSQ: %d,%d", &rssi, &ber) == 2) {
	            // Validate RSSI range
	            bool valid =
	                (rssi >= 15 && rssi <= 31) ||  // LTE acceptable range
	                (rssi == 191);                 // TD-SCDMA excellent signal
	
	            if (valid) {
	                printf("Signal strength OK (RSSI=%d)\n", rssi);
	                signal_good = true;
	                break;
	            } else {
	                ESP_LOGW(TAG_GSM, "Weak signal (RSSI=%d), retrying...", rssi);
	                vTaskDelay(pdMS_TO_TICKS(2000)); 
	                continue;
	            }
	        } else {
	            ESP_LOGE(TAG_GSM, "Unexpected CSQ response: %s", response);
	            vTaskDelay(pdMS_TO_TICKS(2000)); 
	            continue;
	        }	 
	    }
	}
	if (!signal_good) {
	    ESP_LOGE(TAG_GSM, "Unstable or weak GSM signal after 10 attempts. Aborting OTA sequence!");
	    return false;
	}

	// Quectel GSM PDP Context configuration and PDP Activation
	char apncommand[60];
	sprintf(apncommand, "AT+QICSGP=1,1,\"%s\",\"\",\"\",1", APN);
    bool success = false;
    for (int attempt = 0; attempt < RETRIES; attempt++) {
        if (!send_at_command(apncommand, "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QIACT=1", "OK", RETRIES, 3000) || 
            !send_at_command("AT+QIACT?", "+QIACT", RETRIES, CMD_DELAY_MS)) {
            ESP_LOGW(TAG_GSM, "GSM setup failed on attempt %d. Retrying...", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1))); 
            continue;
        } else {
            success = true;
            ESP_LOGI(TAG_GSM, "GSM setup (PDP Configuration and Activation) successful!");
            break; 
        }
    }
    if (!success) {
        ESP_LOGE(TAG_GSM, "GSM setup failed after %d retries. Shutting down and exiting...", RETRIES);
        return false;
    }
  return true;
}


/**
*  @brief Deactivate pdp context
*/
bool deactivate_pdp() {
	const int retries_pdp_deactivate = 2;
	bool pdp_deactivated = false;
	for (int attempt = 0; attempt < retries_pdp_deactivate; attempt++) {
	    if (!send_at_command("AT+QIDEACT=1", "OK", RETRIES, CMD_DELAY_MS)) {
	        ESP_LOGW(TAG_GSM, "Failed to deactivate PDP context on attempt %d, retrying...", attempt + 1);
	        vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	    } else {
	        ESP_LOGI(TAG_GSM, "PDP context deactivated successfully on attempt %d.", attempt + 1);
	        pdp_deactivated = true;
	        break;
	    }    
	} if (!pdp_deactivated) {
	    ESP_LOGE(TAG_GSM, "Failed to deactivate PDP context after %d attempts.", retries_pdp_deactivate);
	    return false;
	}
  return true;	
}

/**
*  @brief Power down modem
*/
void powerdown_modem() {
    const int retries_modem_power_down = 2;
    bool modem_powered_down = false;
    for (int attempt = 0; attempt < retries_modem_power_down; attempt++) {
        if (!send_at_command("AT+QPOWD=1", "OK", RETRIES, CMD_DELAY_MS)) {
            ESP_LOGW(TAG_GSM, "Failed to AT power down modem on attempt %d, retrying...\n", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
        } else {
            modem_powered_down = true;
            printf("Modem powered down successfully!\n");
            break;
        }
        
    } if (!modem_powered_down) {
        ESP_LOGE(TAG_GSM, "AT command power down failed after %d retries. Initiating Hardware Power Down...\n", retries_modem_power_down);
        hardware_poweroff();
    }
    
	// Clean up
	uart_driver_delete(MODEM_UART_NUM);	
  return;	
}

/*******************************************************************************************************************************
 FUNCTIONS: THINGSBOARD FIRMWARE GET OTA INFO
********************************************/
bool get_thingsboard_fw_info(void) { 
	// Power on and initilaize modem
	if (!poweron_modem() || !activate_pdp()) {
	    ESP_LOGE(TAG_GSM, "Modem initialization failed!");
	    powerdown_modem();
        return false;
    } else {
        printf("Modem initialized successfully!\n");
    }	
    
	/**
	*  @brief Setting API URL for ThingsBoard Firmware Details Request
	*/
	// Derive the ThingsBoard device token from ESP32 Chip ID
	otagetChipIdString(device_token, sizeof(device_token)); 
		
    // Build the ThingsBoard API URL
    char url[512];           
    snprintf(url, sizeof(url),
             "http://%s/api/v1/%s/attributes?sharedKeys=fw_title,fw_version,fw_checksum,fw_checksum_algorithm",
             THINGSBOARD_HOST, device_token);                     
     ESP_LOGI(TAG_GSM, "API URL: %s", url);
	 
    char httpget_cmd[100];
    sprintf(httpget_cmd, "AT+QHTTPURL=%d,80", strlen(url));
	
	const int retries_http_url = 3;
	bool http_url_set = false;
	for (int attempt = 0; attempt < retries_http_url; attempt++) {
	    if (!send_at_command(httpget_cmd,"CONNECT", RETRIES, 3000)) {
	        ESP_LOGW(TAG_GSM, "Failed to set API url on attempt %d, retrying...", attempt + 1);
	        vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	        continue;
	    } else {
	        uart_write_bytes(MODEM_UART_NUM, url, strlen(url));
	        vTaskDelay(pdMS_TO_TICKS(2000)); 

	        // IMPORTANT: Wait for the final OK response after URL input
	        if (!send_at_command("", "OK", RETRIES, CMD_DELAY_MS)) {
	            ESP_LOGW(TAG_GSM, "URL configuration not confirmed with OK, retrying...");
	            continue;
	        }

	        http_url_set = true;
	        ESP_LOGI(TAG_GSM, "API url set successfully on attempt %d.", attempt + 1);
	        break;
	    }	    
	} if (!http_url_set) {
	    ESP_LOGE(TAG_GSM, "Failed to set API url after %d attempts. Shutting down and exiting...", retries_http_url);
	    powerdown_modem();
	    return false;
	}

	/**
	 * @brief Quectel HTTPS GET Request and Response Read
	 */
	bool get_success = false;
	for (int attempt = 0; attempt < RETRIES; attempt++) {
        if (!send_at_command("AT+QHTTPGET=80", "+QHTTPGET:", RETRIES, 10000)) {
	        ESP_LOGW(TAG_GSM, "HTTPS GET request failed on attempt %d. Retrying...", attempt + 1);
	        vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	        continue;
        } else {
			// Once GET succeeds, try reading response
			for (int attempt = 0; attempt < RETRIES; attempt++) {
		        if (!send_at_command("AT+QHTTPREAD=80", "+QHTTPREAD: 0", RETRIES, 10000)) {
			        ESP_LOGW(TAG_GSM, "HTTPS Read response failed on attempt %d. Retrying...", attempt + 1);
			        vTaskDelay(pdMS_TO_TICKS(2000));
			        continue;
		        } else {
				    get_success = true;
				    ESP_LOGI(TAG_GSM, "HTTPS GET + READ completed successfully on attempt %d.", attempt + 1);				   
				    //printf("Thingsboard Response: %s", tb_fw_info);

	                // Process HTTPS JSON Response and Parse Firmware Info
	                char *json_start = strchr(tb_fw_info, '{');
	                char *json_end = strrchr(tb_fw_info, '}');	                
	                if (!json_start || !json_end || json_end <= json_start) {
	                    ESP_LOGE(TAG_GSM, "Failed to locate JSON in response!");
	                    ESP_LOGE(TAG_GSM, "Full response length: %d", strlen(tb_fw_info));
	                    ESP_LOGE(TAG_GSM, "First 100 chars: %.100s", tb_fw_info);
	                    powerdown_modem();
	                    return false;
	                }	                
	                // Calculate JSON length and copy to clean buffer
	                size_t json_length = json_end - json_start + 1;
	                char json_clean[json_length + 1];
	                memcpy(json_clean, json_start, json_length);
	                json_clean[json_length] = '\0';	                
	                ESP_LOGI(TAG_GSM, "Cleaned JSON: %s", json_clean);	
	                // Parse JSON
	                cJSON *root = cJSON_Parse(json_clean);
	                if (!root) {
	                    const char *error_ptr = cJSON_GetErrorPtr();
	                    if (error_ptr != NULL) {
	                        ESP_LOGE(TAG_GSM, "JSON parsing error before: %s", error_ptr);
	                    }
	                    ESP_LOGE(TAG_GSM, "Failed to parse JSON!");
	                    powerdown_modem();
	                    return false;
	                }				
				    // TB sometimes nests shared attributes under "shared"
				    cJSON *shared = cJSON_GetObjectItem(root, "shared");
				    cJSON *obj = shared ? shared : root;
				
				    cJSON *title = cJSON_GetObjectItem(obj, "fw_title");
				    cJSON *version = cJSON_GetObjectItem(obj, "fw_version");
				    cJSON *checksum = cJSON_GetObjectItem(obj, "fw_checksum");
				    cJSON *algorithm = cJSON_GetObjectItem(obj, "fw_checksum_algorithm");
				
				    memset(&tb_info, 0, sizeof(tb_fw_info_t));
				
				    if (cJSON_IsString(title)) strncpy(tb_info.fw_title, title->valuestring, sizeof(tb_info.fw_title) - 1);
				    if (cJSON_IsString(version)) strncpy(tb_info.fw_version, version->valuestring, sizeof(tb_info.fw_version) - 1);
				    if (cJSON_IsString(checksum)) strncpy(tb_info.fw_checksum, checksum->valuestring, sizeof(tb_info.fw_checksum) - 1);
				    if (cJSON_IsString(algorithm)) strncpy(tb_info.fw_checksum_algorithm, algorithm->valuestring, sizeof(tb_info.fw_checksum_algorithm) - 1);
				
					cJSON_Delete(root);
				
				    ESP_LOGI(TAG_GSM, "Fetched TB Firmware Info:");
				    ESP_LOGI(TAG_GSM, " Title: %s", tb_info.fw_title);
				    ESP_LOGI(TAG_GSM, " Version: %s", tb_info.fw_version);
				    ESP_LOGI(TAG_GSM, " Checksum: %s", tb_info.fw_checksum);
				    ESP_LOGI(TAG_GSM, " Algorithm: %s", tb_info.fw_checksum_algorithm);
													
					// --- Compare with current firmware version and title ---
					if (strlen(tb_info.fw_title) > 0 && strlen(tb_info.fw_version) > 0) {					
					    bool title_match = strcmp(tb_info.fw_title, FIRMWARE_TITLE) == 0;
					    float current_ver = 0.0f;
					    float remote_ver  = 0.0f;
					    bool valid_format = true;					
					    // Parse current version
					    if (FIRMWARE_VERSION[0] == 'v') {
					        current_ver = atof(&FIRMWARE_VERSION[1]);
					    } else {
					        printf("Wrong Current Firmware format! Aborting OTA!\n");
					        valid_format = false;
					    }					
					    // Parse remote version
					    if (tb_info.fw_version[0] == 'v') {
					        remote_ver = atof(&tb_info.fw_version[1]);
					    } else {
					        printf("Wrong Remote Firmware format! Aborting OTA!\n");
					        valid_format = false;
					    }					
					    // Abort if format is invalid
					    if (!valid_format) {
					        ota_required = false;
					    }
					    else if (title_match && (remote_ver > current_ver)) {
					        ota_required = true;
					        printf("New firmware available!\n");
					        printf("Current: %s (%s) | Remote: %s (%s)\n",
					               FIRMWARE_TITLE, FIRMWARE_VERSION,
					               tb_info.fw_title, tb_info.fw_version);
					    }
					    else {
					        ota_required = false;
					
					        if (!title_match) {
					            printf("Firmware Title Mismatch! Current: %s | Remote: %s Aborting OTA!\n",
					                   FIRMWARE_TITLE, tb_info.fw_title);
					        }
					        else if (remote_ver < current_ver) {
					            printf("Running Firmware is newer. Current: %s | Remote: %s Aborting OTA!\n",
					                   FIRMWARE_VERSION, tb_info.fw_version);
					        }
					        else { // equal
					            printf("Firmware up to date. Current: %s | Remote: %s\n\n",
					                   FIRMWARE_VERSION, tb_info.fw_version);
					        }
					    }
					
					} else {
					    ota_required = false;
					    ESP_LOGE("OTA", "Incomplete firmware attributes received from ThingsBoard.\n");
					}									
				    break;
		        }
		    }
		   break;
        }
    }
    if (!get_success) {
	    ESP_LOGE(TAG_GSM, "HTTPS GET failed after %d attempts. Shutting down...", RETRIES);
	    powerdown_modem();
        return false;
    }	
	
	// Deinitialize and power off modem
	if (!deactivate_pdp()) {
	    ESP_LOGE(TAG_GSM, "Modem deinitialization failed!");
	    powerdown_modem();
    } else {
        printf("Modem deinitialized successfully!\n");
        powerdown_modem();     
    }
  return true;	
}
/*******************************************************************************************************************************/



/********************************************************************************************************************************
FUNCTIONS: OTA FUNCTIONS
************************/
/**
 * Initialize GSM OTA update
 */
esp_err_t gsm_ota_begin(gsm_ota_handle_t *handle, int firmware_size, bool bulk_erase) {
    if (handle == NULL || firmware_size <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize state
    handle->state = ESP_HTTPS_OTA_INIT;
    
    // Get OTA update partition
    handle->update_partition = esp_ota_get_next_update_partition(NULL);
    if (handle->update_partition == NULL) {
        ESP_LOGE(TAG_GSM, "No OTA update partition found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_GSM, "Writing to partition subtype %d at offset 0x%" PRIx32,
             handle->update_partition->subtype, handle->update_partition->address);

    // Allocate buffer for streaming
    handle->buffer_size = 4096; // 4KB chunks
    handle->ota_buffer = malloc(handle->buffer_size);
    if (handle->ota_buffer == NULL) {
        ESP_LOGE(TAG_GSM, "Failed to allocate OTA buffer");
        return ESP_ERR_NO_MEM;
    }

    handle->file_size = firmware_size;
    handle->bytes_received = 0;
    handle->bulk_flash_erase = bulk_erase;

    ESP_LOGI(TAG_GSM, "GSM OTA begin - Firmware size: %d bytes", firmware_size);
    ESP_LOGI(TAG_GSM, "Writing to partition: %s", handle->update_partition->label);

    handle->state = ESP_HTTPS_OTA_BEGIN;
    return ESP_OK;
}

/**
 * Read data from GSM modem with timeout handling
 */
int gsm_ota_read_data(gsm_ota_handle_t *handle, uint8_t *buffer, int max_len, int timeout_ms) {
    uint64_t start_time = esp_timer_get_time();
    const int64_t timeout_us = timeout_ms * 1000;
    
    while ((esp_timer_get_time() - start_time) < timeout_us) {
        int bytes_available = 0;
        if (uart_get_buffered_data_len(MODEM_UART_NUM, (size_t*)&bytes_available) == ESP_OK && bytes_available > 0) {
            int to_read = MIN(bytes_available, max_len);
            int bytes_read = uart_read_bytes(MODEM_UART_NUM, buffer, to_read, pdMS_TO_TICKS(10));
            
            if (bytes_read > 0) {
                return bytes_read;
            }
        }
        
        // Add a small delay to prevent busy waiting, but make it shorter
        vTaskDelay(pdMS_TO_TICKS(10)); // 10 mS responsive checking
    }
    
    return 0; // Timeout
}

/**
 * Verify chip ID from image header
 */
esp_err_t gsm_ota_verify_chip_id(const void *data) {
    esp_image_header_t *image_header = (esp_image_header_t *)data;
    
    if (image_header->chip_id != CONFIG_IDF_FIRMWARE_CHIP_ID) {
        ESP_LOGE(TAG_GSM, "Mismatch chip id, expected %d, found %d", 
                 CONFIG_IDF_FIRMWARE_CHIP_ID, image_header->chip_id);
        return ESP_ERR_INVALID_VERSION;
    }
    return ESP_OK;
}

/**
 * Write data to OTA partition with progress reporting
 */
esp_err_t gsm_ota_write_data(gsm_ota_handle_t *handle, const void *data, size_t len) {
    esp_err_t err = esp_ota_write(handle->update_handle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_GSM, "OTA write failed: %s", esp_err_to_name(err));
        return err;
    }

    int previous_received = handle->bytes_received;
    handle->bytes_received += len;

    // Calculate progress percentages
    int previous_progress = (previous_received * 100) / handle->file_size;
    int current_progress = (handle->bytes_received * 100) / handle->file_size;

    // Log progress every 10% increment or when complete
    if ((current_progress / 10) > (previous_progress / 10) || 
        (handle->bytes_received == handle->file_size)) {
        ESP_LOGI(TAG_GSM, "Download progress: %d%% (%d/%d bytes)", 
                current_progress, handle->bytes_received, handle->file_size);
    }

    return ESP_OK;
}

/**
 * Stream firmware data to OTA partition
 */
esp_err_t gsm_ota_perform(gsm_ota_handle_t *handle) {
    if (handle == NULL || handle->ota_buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;

    switch (handle->state) {
		case ESP_HTTPS_OTA_BEGIN: {
		    const int erase_size = handle->bulk_flash_erase ? 
		        (handle->file_size > 0 ? handle->file_size : OTA_SIZE_UNKNOWN) : 
		        OTA_WITH_SEQUENTIAL_WRITES;
		
		    err = esp_ota_begin(handle->update_partition, erase_size, &handle->update_handle);
		    if (err != ESP_OK) {
		        ESP_LOGE(TAG_GSM, "OTA begin failed: %s", esp_err_to_name(err));
		        return err;
		    }
		
		    handle->bytes_received = 0;
		    printf("Starting firmware streaming via GSM...\n");
		
		    // Clear UART buffer
		    int bytes_available = 0;
		    uart_get_buffered_data_len(MODEM_UART_NUM, (size_t*)&bytes_available);
		    if (bytes_available > 0) {
		        uint8_t *tmp = malloc(bytes_available);
		        if (tmp) {
		            uart_read_bytes(MODEM_UART_NUM, tmp, bytes_available, pdMS_TO_TICKS(100));
		            free(tmp);
		        }
		    }
		
		    // Send command
		    const char *qhttpread_cmd = "AT+QHTTPREAD=80\r\n";
		    uart_write_bytes(MODEM_UART_NUM, qhttpread_cmd, strlen(qhttpread_cmd));
		
		    ESP_LOGI(TAG_GSM, "Waiting for CONNECT and streaming data...");
		
		    bool connect_found = false;
		    uint64_t start_time = esp_timer_get_time();
		
		    while (!connect_found) {
		
		        // Timeout safety
		        if ((esp_timer_get_time() - start_time) > 10000000ULL) { // 10s
		            ESP_LOGE(TAG_GSM, "Timeout waiting for CONNECT");
		            return ESP_FAIL;
		        }
		
		        int available = 0;
		        uart_get_buffered_data_len(MODEM_UART_NUM, (size_t*)&available);
		
		        if (available == 0) {
		            vTaskDelay(pdMS_TO_TICKS(10));
		            continue;
		        }
		
		        int to_read = (available < handle->buffer_size) ? available : handle->buffer_size;
		        int bytes_read = uart_read_bytes(MODEM_UART_NUM, handle->ota_buffer, to_read, pdMS_TO_TICKS(20));
		
		        if (bytes_read <= 0) continue;
		
		        // Search CONNECT
		        char *connect_ptr = memmem(handle->ota_buffer, bytes_read, "CONNECT", 7);
		
		        if (connect_ptr) {
		            connect_found = true;
		            ESP_LOGI(TAG_GSM, "CONNECT detected");
		
		            uint8_t *firmware_start = (uint8_t *)(connect_ptr + 7);
		
		            // Skip CR/LF
		            while (firmware_start < (handle->ota_buffer + bytes_read) &&
		                   (*firmware_start == '\r' || *firmware_start == '\n')) {
		                firmware_start++;
		            }
		
		            int firmware_bytes = (handle->ota_buffer + bytes_read) - firmware_start;
		
		            if (firmware_bytes > 0) {
		                ESP_LOGI(TAG_GSM, "Initial firmware bytes: %d", firmware_bytes);
		
		                if (firmware_bytes >= 4) {
		                    ESP_LOGI(TAG_GSM, "First bytes: 0x%02X 0x%02X 0x%02X 0x%02X",
		                             firmware_start[0], firmware_start[1],
		                             firmware_start[2], firmware_start[3]);
		                }
		
		                err = gsm_ota_write_data(handle, firmware_start, firmware_bytes);
		                if (err != ESP_OK) return err;
		            }
		
		        } else {
		            // CONNECT not yet found → discard (still header/noise)
		            continue;
		        }
		    }
		
		    handle->state = ESP_HTTPS_OTA_IN_PROGRESS;
		    __attribute__((fallthrough));
		}
  
        case ESP_HTTPS_OTA_IN_PROGRESS: {
            uint64_t last_data_time = esp_timer_get_time();
            const uint64_t no_data_timeout_us = 70000000ULL; // 70 seconds

            printf("Streaming firmware... expected size: %d bytes\n", handle->file_size);

            while (handle->bytes_received < handle->file_size) {

                // Timeout protection
                if ((esp_timer_get_time() - last_data_time) > no_data_timeout_us) {
                    ESP_LOGE(TAG_GSM, "Timeout - no data received for 70 seconds");
                    return ESP_ERR_TIMEOUT;
                }

                // Calculate remaining bytes
                int remaining = handle->file_size - handle->bytes_received;
                int bytes_to_read = (remaining < handle->buffer_size) ? remaining : handle->buffer_size;

                // Read from UART
                int bytes_read = gsm_ota_read_data(handle, handle->ota_buffer, bytes_to_read, 100);

                if (bytes_read > 0) {
                    last_data_time = esp_timer_get_time();

                    // Write directly to OTA
                    err = gsm_ota_write_data(handle, handle->ota_buffer, bytes_read);
                    if (err != ESP_OK) {
                        return err;
                    }

                } else {
                    // Small delay if no data
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }

            printf("Firmware download complete (%d/%d bytes)\n",
                    handle->bytes_received, handle->file_size);

            handle->state = ESP_HTTPS_OTA_SUCCESS;
            break;
        }
        
        default:
            ESP_LOGE(TAG_GSM, "Invalid GSM OTA state: %d", handle->state);
            return ESP_ERR_INVALID_STATE;
    }
    return err;
}

/**
 * Check if OTA download is complete
 */
bool gsm_ota_is_complete_data_received(gsm_ota_handle_t *handle) {
    if (handle == NULL) {
        return false;
    }
    return (handle->bytes_received >= handle->file_size);
}

/**
 * Get number of bytes received
 */
int gsm_ota_get_bytes_received(gsm_ota_handle_t *handle) {
    if (handle == NULL) {
        return -1;
    }
    return handle->bytes_received;
}

/**
 * Complete GSM OTA update
 */
esp_err_t gsm_ota_finish(gsm_ota_handle_t *handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;

    // Free buffer
    if (handle->ota_buffer) {
        free(handle->ota_buffer);
        handle->ota_buffer = NULL;
    }

    // End OTA update if we were in progress or success state
    if (handle->state == ESP_HTTPS_OTA_IN_PROGRESS || handle->state == ESP_HTTPS_OTA_SUCCESS) {
        err = esp_ota_end(handle->update_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_GSM, "OTA end failed: %s", esp_err_to_name(err));
            // Don't return yet, we still need to clean up
        }
    }

    // Set boot partition only if download was successful and OTA end was successful
    if (handle->bytes_received == handle->file_size && err == ESP_OK) {
        err = esp_ota_set_boot_partition(handle->update_partition);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_GSM, "Failed to set boot partition: %s", esp_err_to_name(err));
        } else {
            printf("GSM OTA update successful! Device will boot from new firmware on restart.\n");
        }
    } else {
        ESP_LOGE(TAG_GSM, "Incomplete download: %d/%d bytes", handle->bytes_received, handle->file_size);
        if (err == ESP_OK) {
            err = ESP_FAIL;
        }
    }

    return err;
}

/**
 * Abort GSM OTA update
 */
esp_err_t gsm_ota_abort(gsm_ota_handle_t *handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->ota_buffer) {
        free(handle->ota_buffer);
        handle->ota_buffer = NULL;
    }

    if (handle->state == ESP_HTTPS_OTA_IN_PROGRESS || handle->state == ESP_HTTPS_OTA_SUCCESS) {
        esp_ota_abort(handle->update_handle);
    }

    ESP_LOGI(TAG_GSM, "GSM OTA update aborted");

    return ESP_OK;
}

/**
 *	@brief Update the Firmware Dashboard for OTA Update Tracking with Pre Stream Updates
 */
bool tb_report_pre_stream_status(void)
{
	// Power on and initilaize modem
	if (!poweron_modem() || !activate_pdp()) {
	    ESP_LOGE(TAG_GSM, "Modem initialization failed!");
	    powerdown_modem();
        return false;
    } else {
        printf("Modem initialized successfully!\n");
    }	

	/**
	*  @brief HTTP(S) Configuration
	*/
	bool sequence_retry = false;
	for (int attempt = 0; attempt < 3; attempt++) {
        if (!send_at_command("AT+QHTTPCFG=\"contextid\",1", "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QSSLCFG=\"sslversion\",1,4", "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QSSLCFG=\"ciphersuite\",1,0xFFFF", "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QSSLCFG=\"seclevel\",1,0", "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QHTTPCFG=\"requestheader\",1", "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QHTTPCFG=\"responseheader\",1", "OK", RETRIES, CMD_DELAY_MS)) {
            ESP_LOGE(TAG_GSM, "http(s) configuration failed, continuing...\n");
            continue;
        } else {
            printf("http(s) configuration successful!\n");
            sequence_retry = true;
            break;
        }
    }
    if (!sequence_retry) {
        ESP_LOGE(TAG_GSM, "Max retries for http(s) configuration sequence reached. Shutting down and exiting...\n");
        powerdown_modem();
        return false;
    }

	
    // --- URL for ThingsBoard API ---
    char url_telemetry[256];
    snprintf(url_telemetry, sizeof(url_telemetry), "https://%s/api/v1/%s/telemetry", THINGSBOARD_HOST, device_token);
    
    char httpget_cmd[100];
    sprintf(httpget_cmd, "AT+QHTTPURL=%d,80", strlen(url_telemetry));
	
	const int retries_http_url = 3;
	bool http_url_set = false;
	for (int attempt = 0; attempt < retries_http_url; attempt++) {
	    if (!send_at_command(httpget_cmd,"CONNECT", RETRIES, 3000)) {
	        ESP_LOGW(TAG_GSM, "Failed to set Telemetry url on attempt %d, retrying...", attempt + 1);
	        vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	        continue;
	    } else {
	        uart_write_bytes(MODEM_UART_NUM, url_telemetry, strlen(url_telemetry));
	        vTaskDelay(pdMS_TO_TICKS(2000)); 

	        // IMPORTANT: Wait for the final OK response after URL input
	        if (!send_at_command("", "OK", RETRIES, CMD_DELAY_MS)) {
	            ESP_LOGW(TAG_GSM, "URL configuration not confirmed with OK, retrying...");
	            continue;
	        }

	        http_url_set = true;
	        ESP_LOGI(TAG_GSM, "Telemetry url set successfully on attempt %d.", attempt + 1);
	        break;
	    }	    
	} 
    
    if (!http_url_set) {
	    ESP_LOGE(TAG_GSM, "Failed to set Telemetry url after %d attempts. Shutting down and exiting...", retries_http_url);
 	    powerdown_modem();
        return false;
	}
    
    char *state = NULL;
    int progress = 0;

	for (progress = 20; progress <= 80; progress += 20) {
     	state="DOWNLOADING";        
        // --- TELEMETRY Payload (dashboard live updates) ---
        cJSON *telemetry = cJSON_CreateObject();
        if (!telemetry) {
            ESP_LOGE(TAG_GSM,"Failed to create telemetry object");
            powerdown_modem();
            return false;
        }

        // Core OTA progress/state updates
        if (state) cJSON_AddStringToObject(telemetry, "fw_state", state);   // DOWNLOADING, VERIFYING, SUCCESS, FAILED
        cJSON_AddNumberToObject(telemetry, "fw_progress", progress);        // 0–100% progress

        // Current firmware metadata
        cJSON_AddStringToObject(telemetry, "current_fw_title", FIRMWARE_TITLE);
        cJSON_AddStringToObject(telemetry, "current_fw_version", FIRMWARE_VERSION);
        // Target firmware metadata
        cJSON_AddStringToObject(telemetry, "target_fw_title", tb_info.fw_title);
        cJSON_AddStringToObject(telemetry, "target_fw_version", tb_info.fw_version); 

        // --- Convert to string ---
        char *telemetry_str = cJSON_PrintUnformatted(telemetry);
        if (!telemetry_str) {
            ESP_LOGE(TAG_GSM, "Failed to serialize telemetry JSON");
            cJSON_Delete(telemetry);
            powerdown_modem();
            return false;
        } 

        char postRequest[BUF_SIZE];
        snprintf(postRequest, BUF_SIZE,
                "POST /api/v1/%s/telemetry HTTP/1.1\r\n"
                "Host: demo.thingsboard.io\r\n"
                "Content-Type: application/json\r\n"
                "Content-Length: %d\r\n\r\n%s",
                device_token,
                (int)strlen(telemetry_str),
                telemetry_str);
                
        int postRequestLen = strlen(postRequest);
        snprintf(response, BUF_SIZE, "AT+QHTTPPOST=%d,80,80", postRequestLen);
        bool http_post_success = false;

        for (int attempt = 0; attempt < 3; attempt++) {
            if (!send_at_command(response, "CONNECT", RETRIES, 3000)) {
                ESP_LOGW(TAG_GSM, "HTTP POST attempt %d failed", attempt + 1);
                continue;
            }
            uart_write_bytes(MODEM_UART_NUM, postRequest, postRequestLen);
            vTaskDelay(pdMS_TO_TICKS(1000));
            uart_write_bytes(MODEM_UART_NUM, "\x1A", 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            http_post_success = true;
            break;
        }
        // --- Cleanup ---
        cJSON_Delete(telemetry);
        free(telemetry_str);
        
        if (!http_post_success) {
            powerdown_modem();
            return false;
        }	
               
        // --- Logging ---
        ESP_LOGI(TAG_GSM, "Reported OTA state: %s | Progress: %d%%", state ? state : "N/A", progress);
    }

        // Deinitialize and power off modem
        if (!deactivate_pdp()) {
            ESP_LOGE(TAG_GSM, "Modem deinitialization failed!");
            powerdown_modem();
        } else {
            printf("Modem deinitialized successfully!\n");
            powerdown_modem();     
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
  return true;
}



/**
 *	@brief Update the Firmware Dashboard for OTA Update Tracking with Post Stream Updates
 */
bool tb_report_post_stream_status(int ota)
{
	// Power on and initilaize modem
	if (!poweron_modem() || !activate_pdp()) {
	    ESP_LOGE(TAG_GSM, "Modem initialization failed!");
	    powerdown_modem();
        return false;
    } else {
        printf("Modem initialized successfully!\n");
    }	
	
	/**
	*  @brief HTTP(S) Configuration
	*/
	bool sequence_retry = false;
	for (int attempt = 0; attempt < 3; attempt++) {
        if (!send_at_command("AT+QHTTPCFG=\"contextid\",1", "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QSSLCFG=\"sslversion\",1,4", "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QSSLCFG=\"ciphersuite\",1,0xFFFF", "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QSSLCFG=\"seclevel\",1,0", "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QHTTPCFG=\"requestheader\",1", "OK", RETRIES, CMD_DELAY_MS) ||
            !send_at_command("AT+QHTTPCFG=\"responseheader\",1", "OK", RETRIES, CMD_DELAY_MS)) {
            ESP_LOGE(TAG_GSM, "http(s) configuration failed, continuing...\n");
            continue;
        } else {
            printf("http(s) configuration successful!\n");
            sequence_retry = true;
            break;
        }
    }
    if (!sequence_retry) {
        ESP_LOGE(TAG_GSM, "Max retries for http(s) configuration sequence reached. Shutting down and exiting...\n");
        powerdown_modem();
        return false;
    }	
	
	
    // --- URL for ThingsBoard API ---
    char url_telemetry[256];
    snprintf(url_telemetry, sizeof(url_telemetry), "https://%s/api/v1/%s/telemetry", THINGSBOARD_HOST, device_token);
    
    char httpget_cmd[100];
    sprintf(httpget_cmd, "AT+QHTTPURL=%d,80", strlen(url_telemetry));
	
	const int retries_http_url = 3;
	bool http_url_set = false;
	for (int attempt = 0; attempt < retries_http_url; attempt++) {
	    if (!send_at_command(httpget_cmd,"CONNECT", RETRIES, 3000)) {
	        ESP_LOGW(TAG_GSM, "Failed to set Telemetry url on attempt %d, retrying...", attempt + 1);
	        vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	        continue;
	    } else {
	        uart_write_bytes(MODEM_UART_NUM, url_telemetry, strlen(url_telemetry));
	        vTaskDelay(pdMS_TO_TICKS(2000)); 

	        // IMPORTANT: Wait for the final OK response after URL input
	        if (!send_at_command("", "OK", RETRIES, CMD_DELAY_MS)) {
	            ESP_LOGW(TAG_GSM, "URL configuration not confirmed with OK, retrying...");
	            continue;
	        }

	        http_url_set = true;
	        ESP_LOGI(TAG_GSM, "Telemetry url set successfully on attempt %d.", attempt + 1);
	        break;
	    }	    
	} 
    
    if (!http_url_set) {
	    ESP_LOGE(TAG_GSM, "Failed to set Telemetry url after %d attempts. Shutting down and exiting...", retries_http_url);
 	    powerdown_modem();
        return false;
	}

    char *state = NULL;
    int progress = 0;

    for (int i = 0; i < 2; i++) {
        // --- TELEMETRY Payload (dashboard live updates) ---
        cJSON *telemetry = cJSON_CreateObject();
        if (!telemetry) {
            ESP_LOGE(TAG_GSM,"Failed to create telemetry object");
            powerdown_modem();
            return false;
        }

        if(i == 0) {
            state = "VERIFYING";
            progress = 90; 
        } else if (i == 1) {
            if (ota == 1) {
                state = "SUCCESS";
                progress = 100;
            } else if (ota == 0) {
                state = "FAILED";
                progress = 99;                
            }
        }

        // Core OTA progress/state updates
        if (state) cJSON_AddStringToObject(telemetry, "fw_state", state);   // DOWNLOADING, VERIFYING, SUCCESS, FAILED
        cJSON_AddNumberToObject(telemetry, "fw_progress", progress);        // 0–100% progress

        // Current firmware metadata
        cJSON_AddStringToObject(telemetry, "current_fw_title", FIRMWARE_TITLE);
        cJSON_AddStringToObject(telemetry, "current_fw_version", FIRMWARE_VERSION);
        // Target firmware metadata
        cJSON_AddStringToObject(telemetry, "target_fw_title", tb_info.fw_title);
        cJSON_AddStringToObject(telemetry, "target_fw_version", tb_info.fw_version);

        // --- Convert to string ---
        char *telemetry_str = cJSON_PrintUnformatted(telemetry);
        if (!telemetry_str) {
            ESP_LOGE(TAG_GSM, "Failed to serialize telemetry JSON");
            cJSON_Delete(telemetry);
            powerdown_modem();
            return false;
        } 

        char postRequest[BUF_SIZE];
        snprintf(postRequest, BUF_SIZE,
                "POST /api/v1/%s/telemetry HTTP/1.1\r\n"
                "Host: demo.thingsboard.io\r\n"
                "Content-Type: application/json\r\n"
                "Content-Length: %d\r\n\r\n%s",
                device_token,
                (int)strlen(telemetry_str),
                telemetry_str);
                
        int postRequestLen = strlen(postRequest);
        snprintf(response, BUF_SIZE, "AT+QHTTPPOST=%d,80,80", postRequestLen);
        bool http_post_success = false;
        
        for (int attempt = 0; attempt < 3; attempt++) {
            if (!send_at_command(response, "CONNECT", RETRIES, 3000)) {
                ESP_LOGW(TAG_GSM, "HTTP POST attempt %d failed", attempt + 1);
                continue;
            }
            uart_write_bytes(MODEM_UART_NUM, postRequest, postRequestLen);
            vTaskDelay(pdMS_TO_TICKS(1000));
            uart_write_bytes(MODEM_UART_NUM, "\x1A", 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            http_post_success = true;
            break;
        }
        // --- Cleanup ---
        cJSON_Delete(telemetry);
        free(telemetry_str);
        
        if (!http_post_success) {
            powerdown_modem();
            return false;
        }	
               
        // --- Logging ---
        ESP_LOGI(TAG_GSM, "Reported OTA state: %s | Progress: %d%%", state ? state : "N/A", progress);
    }

        // Deinitialize and power off modem
        if (!deactivate_pdp()) {
            ESP_LOGE(TAG_GSM, "Modem deinitialization failed!");
            powerdown_modem();
        } else {
            printf("Modem deinitialized successfully!\n");
            powerdown_modem();     
        }
        vTaskDelay(pdMS_TO_TICKS(3000));	
  return true;
}


/**
 *	Perform secure gsm ota
 */
bool perform_secure_ota_update() {
	// Update OTA dashboard with simulated pre-stream updates
    for (int attempt = 0; attempt < RETRIES; attempt++) {
		if (!tb_report_pre_stream_status()) {
			printf("Pre stream ota dashboard update Failed!\n");
			continue;
		} else {
	        printf("Pre stream ota dashboard update posted.\n\n");
	        break; 
		}
    }

	// Power on and initilaize modem
	if (!poweron_modem() || !activate_pdp()) {
	    ESP_LOGE(TAG_GSM, "Modem initialization failed!");
	    powerdown_modem();
        return false;
    } else {
        printf("Modem initialized successfully!\n");
    }
	  
    // Construct full firmware URL
    char firmware_url[512];
    snprintf(firmware_url, sizeof(firmware_url),
             "https://%s/api/v1/%s/firmware?title=%s&version=%s",
             THINGSBOARD_HOST, device_token, tb_info.fw_title, tb_info.fw_version);            

    ESP_LOGI(TAG_GSM, "Firmware URL: %s", firmware_url);
	
    char httpget_cmd[100];
    sprintf(httpget_cmd, "AT+QHTTPURL=%d,80", strlen(firmware_url));
	
	const int retries_http_url = 3;
	bool http_url_set = false;
	for (int attempt = 0; attempt < retries_http_url; attempt++) {
	    if (!send_at_command(httpget_cmd,"CONNECT", RETRIES, 3000)) {
	        ESP_LOGW(TAG_GSM, "Failed to set API url on attempt %d, retrying...", attempt + 1);
	        vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	        continue;
	    } else {
	        uart_write_bytes(MODEM_UART_NUM, firmware_url, strlen(firmware_url));
	        vTaskDelay(pdMS_TO_TICKS(2000)); 

	        // IMPORTANT: Wait for the final OK response after URL input
	        if (!send_at_command("", "OK", RETRIES, CMD_DELAY_MS)) {
	            ESP_LOGW(TAG_GSM, "URL configuration not confirmed with OK, retrying...");
	            continue;
	        }

	        http_url_set = true;
	        ESP_LOGI(TAG_GSM, "API url set successfully on attempt %d.", attempt + 1);
	        break;
	    }	    
	} if (!http_url_set) {
	    ESP_LOGE(TAG_GSM, "Failed to set API url after %d attempts. Shutting down and exiting...", retries_http_url);
	    powerdown_modem();
	    return false;
	}

	/**
	 * @brief Quectel HTTPS GET Request and Response Read
	 */
    int file_size = 0;
    bool get_success = false;
    for (int attempt = 0; attempt < RETRIES; attempt++) {
        if (!send_at_command("AT+QHTTPGET=80", "+QHTTPGET:", RETRIES, 5000)) {
            ESP_LOGW(TAG_GSM, "HTTPS Firmware Size GET failed on attempt %d", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
            continue;
        } else {      
			// Parse the response to get file size: +QHTTPGET: 0,200,237840
			char *qhttpget_ptr = strstr(response, "+QHTTPGET:");
			if (qhttpget_ptr) {
			    if (sscanf(qhttpget_ptr, "+QHTTPGET: 0,200,%d", &file_size) == 1) {
			        get_success = true;
			        ESP_LOGI(TAG_GSM, "Firmware Size GET successful. File size: %d bytes", file_size);
			        break;
			    } else {
			        ESP_LOGW(TAG_GSM, "Could not parse file size from QHTTPGET response: %s", qhttpget_ptr);
			    }
			} else {
			    ESP_LOGW(TAG_GSM, "QHTTPGET pattern not found in response: %s", response);
			}
        }
    }
    
    if (!get_success || file_size <= 0) {
        ESP_LOGE(TAG_GSM, "Failed to get valid file size");
        powerdown_modem();
        return false;
    }

	/**
	 *  @brief Query current URC port, temporarily disable URCs by routing to USB, and confirm change
	 */
	bool urc_deactivated = false;
	for (int attempt = 0; attempt < RETRIES; attempt++) {
	    if (!send_at_command("AT+QURCCFG=\"urcport\"", "+QURCCFG: \"urcport\"", RETRIES, CMD_DELAY_MS) ||
	        !send_at_command("AT+QURCCFG=\"urcport\",\"usbat\"", "OK", RETRIES, CMD_DELAY_MS) ||
	        !send_at_command("AT+QURCCFG=\"urcport\"", "+QURCCFG: \"urcport\",\"usbat\"", RETRIES, CMD_DELAY_MS)) {     
	        ESP_LOGW(TAG_GSM, "Failed to query, change, and confirm URC port on attempt %d, retrying...", attempt + 1);
	        vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	        continue;
	    } else {
	        ESP_LOGI(TAG_GSM, "URC port successfully queried, changed, and confirmed on attempt %d.", attempt + 1);
	        urc_deactivated = true;
	        break;
	    }    
	}
	if (!urc_deactivated) {
	    ESP_LOGE(TAG_GSM, "Failed to query, change, and confirm URC port after %d attempts.", RETRIES);
	    powerdown_modem();
	    return false;
	}

    // Initialize GSM OTA
    gsm_ota_handle_t gsm_ota_handle;
    esp_err_t err = gsm_ota_begin(&gsm_ota_handle, file_size, true); // Use bulk erase for better performance
    if (err != ESP_OK) {
        ESP_LOGE(TAG_GSM, "GSM OTA begin failed");
        powerdown_modem();
        return false;
    }

    // Perform the OTA update
    err = gsm_ota_perform(&gsm_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_GSM, "GSM OTA perform failed");
        gsm_ota_abort(&gsm_ota_handle);
        powerdown_modem();
        return false;
    } else {
		printf("Firmware download completed successfully!\n");
	}

    // Finish OTA
    err = gsm_ota_finish(&gsm_ota_handle);	
    if (err != ESP_OK) {
        ESP_LOGE(TAG_GSM, "GSM OTA Finish failed");
        powerdown_modem();
        return false;
    } else {
		printf("OTA Finished successfully!\n");
	}
	
	/**
	 *  @brief Restore URC port to UART1 after firmware streaming with robust error handling
	 */
	bool urc_restored = false;
	// STEP 1: Normal retry
	for (int attempt = 0; attempt < RETRIES; attempt++) {
	    if (!send_at_command("AT+QURCCFG=\"urcport\"", "+QURCCFG: \"urcport\"", RETRIES, CMD_DELAY_MS) ||
	        !send_at_command("AT+QURCCFG=\"urcport\",\"uart1\"", "OK", RETRIES, CMD_DELAY_MS) ||
	        !send_at_command("AT+QURCCFG=\"urcport\"", "+QURCCFG: \"urcport\",\"uart1\"", RETRIES, CMD_DELAY_MS)) { 
	        ESP_LOGW(TAG_GSM, "Attempt %d: Failed to query, change, and confirm URC port restoration, retrying...", attempt + 1);
	        vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	        continue;
	    } else {
	        ESP_LOGI(TAG_GSM, "URC port successfully restored to UART1 on attempt %d.", attempt + 1);
	        urc_restored = true;
	        urc_to_usb = false;
	        break;
	    }
	}
	// STEP 2: If failed, soft reset the GSM and retry
	if (!urc_restored) {
	    ESP_LOGW(TAG_GSM, "Initial URC port restoration failed, attempting GSM reset...");
	    gsm_reset();
	    vTaskDelay(pdMS_TO_TICKS(5000)); // wait for module to reboot
	    for (int attempt = 0; attempt < RETRIES; attempt++) {
	        if (!send_at_command("AT+QURCCFG=\"urcport\",\"uart1\"", "OK", RETRIES, CMD_DELAY_MS) ||
	        	!send_at_command("AT+QURCCFG=\"urcport\"", "+QURCCFG: \"urcport\",\"uart1\"", RETRIES, CMD_DELAY_MS)) { 
	            ESP_LOGW(TAG_GSM, "Post-reset attempt %d: Failed to restore URC port, retrying...", attempt + 1);
	            vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	            continue;
	        } else {
	            ESP_LOGI(TAG_GSM, "URC port successfully restored to UART1 after GSM reset (attempt %d).", attempt + 1);
	            urc_restored = true;
	            urc_to_usb = false;
	            break;
	        }
	    }
	}
	// STEP 3: If still failed, power cycle the GSM module
	if (!urc_restored) {
	    ESP_LOGW(TAG_GSM, "URC port restoration still failed, performing modem power cycle...");
	    powerdown_modem();
	    vTaskDelay(pdMS_TO_TICKS(500));
	    poweron_modem();
	    for (int attempt = 0; attempt < RETRIES; attempt++) {
	        if (!send_at_command("AT+QURCCFG=\"urcport\",\"uart1\"", "OK", RETRIES, CMD_DELAY_MS) ||
	        	!send_at_command("AT+QURCCFG=\"urcport\"", "+QURCCFG: \"urcport\",\"uart1\"", RETRIES, CMD_DELAY_MS)) { 
	            ESP_LOGW(TAG_GSM, "Post power-cycle attempt %d: Failed to restore URC port, retrying...", attempt + 1);
	            vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));
	            continue;
	        } else {
	            ESP_LOGI(TAG_GSM, "URC port successfully restored to UART1 after modem power cycle (attempt %d).", attempt + 1);
	            urc_restored = true;
	            urc_to_usb = false;
	            break;
	        }
	    }
	}
	// Final Failure and Escalation
	if (!urc_restored) {
	    urc_to_usb = true;
	    ESP_LOGE(TAG_GSM, "Critical: Failed to restore URC port to UART1 after all recovery attempts.");
        powerdown_modem();
        return false;
	}

	// Deinitialize and power off modem
	if (!deactivate_pdp()) {
	    ESP_LOGE(TAG_GSM, "Modem deinitialization failed!");
	    powerdown_modem();
    } else {
        printf("Modem deinitialized successfully!\n");
        powerdown_modem();     
    }
  return true;
}

/*******************************************************************************************************************************/

// Query OTA flag
bool is_ota_required(void) {
    return ota_required;
}

// Identify running partition
void running_partition() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    printf("Currently running partition: %s\n", running->label); 
    return;
}

/**
*	Query for new firmware and execute ota if found
*/
bool perform_ota() {
    // TB GET attributes and perform ota
    bool tb_fw_info = false;
	for (int attempt = 0; attempt < 3; attempt++) {   
	    if (get_thingsboard_fw_info()) {
	        ESP_LOGI(TAG_GSM, "ThingsBoard Firmware Attributes successfully retrieved.");
	        vTaskDelay(pdMS_TO_TICKS(50));
			    if (is_ota_required()) {
					printf("New Firmware Available, executing ota...\n");
					vTaskDelay(pdMS_TO_TICKS(50));	
					// checking the running slot
					running_partition();	
					vTaskDelay(pdMS_TO_TICKS(50));        
		            bool perform_secure_ota = false;
					for (int attempt = 0; attempt < 3; attempt++) {
						if (!perform_secure_ota_update()) {
						    ESP_LOGE(TAG_GSM, "Ota download failed!");
						    vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1))); // progressive back-off
						    continue;
					    } else {
					        printf("Ota download executed successfully!\n");	
					        perform_secure_ota = true;	

							// Update OTA dashboard with simulated post-stream updates with success
						    for (int attempt = 0; attempt < RETRIES; attempt++) {
								if (!tb_report_post_stream_status(1)) {
									printf("Post stream ota dashboard update Failed!\n");
									continue;
								} else {
							        printf("Post stream ota dashboard update posted.\n\n\n\n");
							        vTaskDelay(pdMS_TO_TICKS(100));
							        break; 
								}
						    }
							vTaskDelay(pdMS_TO_TICKS(100));				        
					        printf("Restarting device to boot new firmware...\n\n\n\n");
					        vTaskDelay(pdMS_TO_TICKS(1000));
					        esp_restart(); // Booting to the new OTA slot				        			          
					        break;      
					    }
					}
				    if (!perform_secure_ota) {
						// Update OTA dashboard with simulated post-stream updates with failure 
					    for (int attempt = 0; attempt < RETRIES; attempt++) {
							if (!tb_report_post_stream_status(0)) {
								printf("Post stream ota dashboard update Failed!\n");
								continue;
							} else {
						        printf("Post stream ota dashboard update posted.\n\n\n\n\n");
						        vTaskDelay(pdMS_TO_TICKS(100));
						        break; 
							}
					    }
						vTaskDelay(pdMS_TO_TICKS(1000));	
				        ESP_LOGE(TAG_GSM, "Ota download failed after %d retries.\n", RETRIES);
				        return false;
				    }	  		        		        
			    } else {
	        		printf("No update needed, continuing normal operation.\n\n"); 
	        		return true;
	    		}
	    	tb_fw_info = true;
	    	break;
	    } else {
	        ESP_LOGE(TAG_GSM, "Failed to fetch firmware attributes from ThingsBoard.");	        
	        continue;
	    }
    }
    if (!tb_fw_info) {
        ESP_LOGE(TAG_GSM, "Failed to fetch firmware attributes from ThingsBoard after %d retries.\n", RETRIES);
        return false;
    }     
    return true;	
}
