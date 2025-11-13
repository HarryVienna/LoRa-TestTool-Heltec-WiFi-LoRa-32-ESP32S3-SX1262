#include "sx1262.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SX1262";

static SemaphoreHandle_t sx1262_mutex = NULL;

// Global variables
static spi_device_handle_t spi_handle;
static sx1262_config_t current_config;
static bool hw_initialized = false;

// Helper functions (Forward Declarations)
static void sx1262_reset(void);
static void sx1262_wait_on_busy(void);

static esp_err_t sx1262_write_command(uint8_t cmd, uint8_t *data, uint8_t len);
static esp_err_t sx1262_read_command(uint8_t cmd, uint8_t *data, uint8_t len);

static esp_err_t sx1262_write_register(uint16_t addr, uint8_t *data, uint8_t len);
static esp_err_t sx1262_read_register(uint16_t addr, uint8_t *data, uint8_t len);

static esp_err_t sx1262_spi_write_general(uint8_t *tx_header, uint8_t tx_header_len, uint8_t *data, uint8_t data_len);
static esp_err_t sx1262_spi_read_general(uint8_t *tx_header, uint8_t tx_header_len, uint8_t *rx_data, uint8_t rx_len);

static esp_err_t sx1262_set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask);
static esp_err_t sx1262_clear_irq_status(uint16_t irq_mask);
static uint16_t sx1262_get_irq_status(void);

// ============================================================================
// PHASE 1: HARDWARE INITIALIZATION
// ============================================================================

esp_err_t sx1262_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Hardware initialization...");

    // Create mutex
    if (sx1262_mutex == NULL) {
        sx1262_mutex = xSemaphoreCreateRecursiveMutex();
        if (sx1262_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Mutex created successfully");
    }

    // GPIO Configuration
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LORA_PIN_RST) | (1ULL << LORA_PIN_BUSY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    // BUSY as Input
    gpio_config(&io_conf);

    // RST as Output
    io_conf.pin_bit_mask = (1ULL << LORA_PIN_RST);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);

    // DIO1 as Input (for Interrupt)
    io_conf.pin_bit_mask = (1ULL << LORA_PIN_DIO1);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    // SPI Bus Configuration
    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_PIN_MISO,
        .mosi_io_num = LORA_PIN_MOSI,
        .sclk_io_num = LORA_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Bus Init failed");
        return ret;
    }

    // SPI Device Configuration
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,  // 1 MHz
        .mode = 0,
        .spics_io_num = LORA_PIN_NSS,
        .queue_size = 7,
        .flags = 0,
        .pre_cb = NULL
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Device Add failed");
        return ret;
    }

    // Hardware Reset
    sx1262_reset();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Wait on BUSY
    sx1262_wait_on_busy();

    // Set Standby Mode
    uint8_t standby_config = 0x01; // STDBY_XOSC (for TCXO)
    ret = sx1262_write_command(SX1262_CMD_SET_STANDBY, &standby_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setting Standby failed");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure DIO3 as TCXO Control (3.3V, 5ms timeout)
    uint8_t tcxo_config[4] = {0x07, 0x00, 0x01, 0x40}; // 320 * 15.625us = 5ms
    ret = sx1262_write_command(SX1262_CMD_SET_DIO3_AS_TCXO_CTRL, tcxo_config, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCXO configuration failed");
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // DIO2 as RF Switch Control
    uint8_t dio2_config = 0x01; // Enable
    ret = sx1262_write_command(SX1262_CMD_SET_DIO2_AS_RF_SWITCH, &dio2_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DIO2 configuration failed");
    }

    // Set Regulator Mode (DC-DC)
    uint8_t regulator_mode = 0x01; // DC-DC + LDO
    ret = sx1262_write_command(SX1262_CMD_SET_REGULATOR_MODE, &regulator_mode, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Regulator Mode failed");
    }

    // Calibrate
    uint8_t calib_param = 0x7F; // All
    ret = sx1262_write_command(SX1262_CMD_CALIBRATE, &calib_param, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Calibration failed");
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Set RxTxFallbackMode (Chip goes back to STDBY_XOSC after TX/RX)
    uint8_t fallback_mode = 0x30; // STDBY_XOSC
    ret = sx1262_write_command(SX1262_CMD_SET_RX_TX_FALLBACK_MODE, &fallback_mode, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RxTx Fallback Mode failed");
        return ret;
    }

    hw_initialized = true;
    ESP_LOGI(TAG, "Hardware initialized successfully");
    
    return ESP_OK;
}

// ============================================================================
// PHASE 2: LORA CONFIGURATION
// ============================================================================

esp_err_t sx1262_configure(const sx1262_config_t *config)
{
    esp_err_t ret = ESP_OK;

    if (!hw_initialized) {
        ESP_LOGE(TAG, "Hardware not initialized! Call sx1262_hw_init() first!");
        return ESP_ERR_INVALID_STATE;
    }

    if (!config || !sx1262_mutex) {
        return ESP_ERR_INVALID_ARG;
    }

    // Take mutex (wait max 5 seconds)
    if (xSemaphoreTakeRecursive(sx1262_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for configure");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Configuring LoRa Parameters...");

    // Copy Config
    memcpy(&current_config, config, sizeof(sx1262_config_t));

    // Standby
    ret = sx1262_standby();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Standby failed");
        goto cleanup;
    }

    // 1. Set Packet Type (LoRa or FSK)
    uint8_t packet_type = (config->modem_mode == SX1262_MODEM_LORA) ? 
                           SX1262_PACKET_TYPE_LORA : SX1262_PACKET_TYPE_GFSK;
    ret = sx1262_write_command(SX1262_CMD_SET_PACKET_TYPE, &packet_type, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setting Packet Type failed");
        goto cleanup;
    }

    // 2. Set RF Frequency
    uint32_t freq_reg = ((uint64_t)config->frequency << 25) / 32000000;
    uint8_t freq_params[4];
    freq_params[0] = (freq_reg >> 24) & 0xFF;
    freq_params[1] = (freq_reg >> 16) & 0xFF;
    freq_params[2] = (freq_reg >> 8) & 0xFF;
    freq_params[3] = freq_reg & 0xFF;
    
    ret = sx1262_write_command(SX1262_CMD_SET_RF_FREQUENCY, freq_params, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setting Frequency failed");
        goto cleanup;
    }

    // 3. Set PA Config

    // Semtech_SX1261_2 V2-2.pdf Chapter 15.2 TxClampConfig fix
    uint8_t tx_clamp_cfg;
    ret = sx1262_read_register(SX1262_REG_TX_CLAMP_CFG, &tx_clamp_cfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TX_CLAMP_CFG Read failed");
        goto cleanup;
    }
    tx_clamp_cfg = tx_clamp_cfg | 0x1E;
    ret = sx1262_write_register(SX1262_REG_TX_CLAMP_CFG, &tx_clamp_cfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TX_CLAMP_CFG Read failed");
        goto cleanup;
    }

    uint8_t pa_config[4];
    int8_t power = config->tx_power;
    
    if (power > 22) power = 22;
    if (power < -9) power = -9;
    
    if (power <= 14) {
        pa_config[0] = 0x02;
        pa_config[1] = 0x02;
        pa_config[2] = 0x00;
        pa_config[3] = 0x01;
    } else if (power <= 17) {
        pa_config[0] = 0x02;
        pa_config[1] = 0x03;
        pa_config[2] = 0x00;
        pa_config[3] = 0x01;
    } else if (power <= 20) {
        pa_config[0] = 0x03;
        pa_config[1] = 0x05;
        pa_config[2] = 0x00;
        pa_config[3] = 0x01;
    } else {
        pa_config[0] = 0x04;
        pa_config[1] = 0x07;
        pa_config[2] = 0x00;
        pa_config[3] = 0x01;
    }
    
    ret = sx1262_write_command(SX1262_CMD_SET_PA_CONFIG, pa_config, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PA Config failed");
        goto cleanup;
    }
    
    // Set OCP
    uint8_t ocp_value;
    if (power >= 20) {
        ocp_value = 0x38;  // 140 mA
    } else if (power >= 17) {
        ocp_value = 0x28;  // 100 mA
    } else {
        ocp_value = 0x18;  // 60 mA
    }
    
    ret = sx1262_write_register(SX1262_REG_OCP_CONFIGURATION, &ocp_value, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Setting OCP failed");
        goto cleanup;
    }

    // 4. Set TX Params
    uint8_t tx_params[2];
    tx_params[0] = power;
    tx_params[1] = 0x04; // 200us ramp
    
    ret = sx1262_write_command(SX1262_CMD_SET_TX_PARAMS, tx_params, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setting TX Params failed");
        goto cleanup;
    }

    // 5. Set Modulation Params
    if (config->modem_mode == SX1262_MODEM_LORA) {
        // LoRa Modulation
        uint8_t mod_params[4];
        mod_params[0] = config->spreading_factor;
        mod_params[1] = config->bandwidth;
        mod_params[2] = config->coding_rate;
        mod_params[3] = (config->spreading_factor >= 11) ? 0x01 : 0x00; // Low Data Rate Optimize
        
        ret = sx1262_write_command(SX1262_CMD_SET_MODULATION_PARAMS, mod_params, 4);
    } else {
        // FSK Modulation
        uint32_t br_reg = (uint32_t)((32.0 * 32000000.0) / config->fsk_bitrate);
        uint32_t fdev_reg = (uint32_t)((config->fsk_fdev * 33554432.0) / 32000000.0);
        
        uint8_t mod_params[8];
        mod_params[0] = (br_reg >> 16) & 0xFF;
        mod_params[1] = (br_reg >> 8) & 0xFF;
        mod_params[2] = br_reg & 0xFF;
        mod_params[3] = config->fsk_shaping;
        mod_params[4] = config->fsk_rx_bw;
        mod_params[5] = (fdev_reg >> 16) & 0xFF;
        mod_params[6] = (fdev_reg >> 8) & 0xFF;
        mod_params[7] = fdev_reg & 0xFF;
        
        ret = sx1262_write_command(SX1262_CMD_SET_MODULATION_PARAMS, mod_params, 8);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Modulation Params failed");
        goto cleanup;
    }

    // 6. Set Packet Params
    if (config->modem_mode == SX1262_MODEM_LORA) {
        // LoRa Packet Parameters
        uint8_t packet_params[6];

        // Semtech_SX1261_2 V2-2.pdf Chapter 6.1.1.1 Preamble length for SF5 and SF6
        uint16_t preamble_length = (config->spreading_factor <= 6) ? 12 : config->preamble_length;

        packet_params[0] = (preamble_length >> 8) & 0xFF;
        packet_params[1] = preamble_length & 0xFF;
        packet_params[2] = config->payload_length == 0 ? 0x00 : 0x01; // Variable/Fixed
        packet_params[3] = config->payload_length;
        packet_params[4] = config->crc_on ? 0x01 : 0x00;
        packet_params[5] = config->iq_inverted ? 0x01 : 0x00;
        
        ret = sx1262_write_command(SX1262_CMD_SET_PACKET_PARAMS, packet_params, 6);
    } else {
        // FSK Packet Parameters
        uint8_t packet_params[9];
        packet_params[0] = (config->preamble_length >> 8) & 0xFF;
        packet_params[1] = config->preamble_length & 0xFF;
        packet_params[2] = 0x04; // Preamble detector length
        packet_params[3] = 0x08; // Sync word length
        packet_params[4] = 0x01; // Address filtering
        packet_params[5] = config->payload_length == 0 ? 0x01 : 0x00; // Variable/Fixed
        packet_params[6] = config->payload_length;
        packet_params[7] = config->crc_on ? 0x01 : 0x00;
        packet_params[8] = 0x00; // Whitening
        
        ret = sx1262_write_command(SX1262_CMD_SET_PACKET_PARAMS, packet_params, 9);
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Packet Params failed");
        goto cleanup;
    }

    // 7. Set Sync Word (LoRa only, optional)
    if (config->modem_mode == SX1262_MODEM_LORA && config->sync_word != 0) {
        uint8_t sync_word_msb = (config->sync_word >> 8) & 0xFF;
        uint8_t sync_word_lsb = config->sync_word & 0xFF;
        
        ret = sx1262_write_register(SX1262_REG_LORA_SYNC_WORD_MSB, &sync_word_msb, 1);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Setting Sync Word MSB failed");
            goto cleanup;
        }
        
        ret = sx1262_write_register(SX1262_REG_LORA_SYNC_WORD_LSB, &sync_word_lsb, 1);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Setting Sync Word LSB failed");
            goto cleanup;
        }
        
        ESP_LOGI(TAG, "Sync Word set: 0x%04X", config->sync_word);
    }

    // 8. Set Buffer Base Address
    uint8_t buffer_params[2] = {0x00, 0x00}; // TX=0, RX=0
    ret = sx1262_write_command(SX1262_CMD_SET_BUFFER_BASE_ADDRESS, buffer_params, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Buffer Base Address failed");
        goto cleanup;
    }

    // 9. Configure IRQ
    ret = sx1262_set_dio_irq_params(SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE | SX1262_IRQ_TIMEOUT,
                                   SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE | SX1262_IRQ_TIMEOUT,
                                   0x0000, 0x0000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IRQ Config failed");
        goto cleanup;
    }

    // 10. Set RX Gain
    if (config->rx_gain_boosted) {
        uint8_t rx_gain_boosted = 0x96;
        ret = sx1262_write_register(SX1262_REG_RX_GAIN, &rx_gain_boosted, 1);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Setting RX Gain Boosted failed");
            goto cleanup;
        } else {
            ESP_LOGI(TAG, "RX Gain: Boosted");
        }
    } else {
        uint8_t rx_gain_power_save = 0x94;
        ret = sx1262_write_register(SX1262_REG_RX_GAIN, &rx_gain_power_save, 1);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Setting RX Gain Power Save failed");
            goto cleanup;
        } else {
            ESP_LOGI(TAG, "RX Gain: Power Save");
        }
    }

    ESP_LOGI(TAG, "Configuration complete: Mode=%s, Freq=%luHz, SF=%d, BW=%d, CR=%d, TX=%ddBm",
             config->modem_mode == SX1262_MODEM_LORA ? "LoRa" : "FSK",
             config->frequency,
             config->spreading_factor,
             config->bandwidth,
             config->coding_rate,
             config->tx_power);    

cleanup:
    // Release mutex
    xSemaphoreGiveRecursive(sx1262_mutex);
    return ret;
}

// ============================================================================
// PHASE 3: COMMUNICATION 
// ============================================================================

esp_err_t sx1262_send(uint8_t *data, uint8_t len)
{
    esp_err_t ret = ESP_OK;

    if (!hw_initialized) {
        ESP_LOGE(TAG, "Hardware not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0 || len > 255 || !sx1262_mutex) {
        return ESP_ERR_INVALID_ARG;
    }

    // Take mutex
    if (xSemaphoreTakeRecursive(sx1262_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for send");
        return ESP_ERR_TIMEOUT;
    }

    // Standby Mode
    ret = sx1262_standby();
    if (ret != ESP_OK) {
        goto cleanup; 
    }

    // Clear IRQ Status
    ret = sx1262_clear_irq_status(0xFFFF);
    if (ret != ESP_OK) {
        goto cleanup; 
    }

    // Write data to buffer
    uint8_t offset = 0x00;
    uint8_t buffer[256];
    buffer[0] = offset;
    memcpy(&buffer[1], data, len);
    
    ret = sx1262_write_command(SX1262_CMD_WRITE_BUFFER, buffer, len + 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Writing buffer failed");
        goto cleanup; 
    }

    // Update packet parameters with current length
    uint8_t packet_params[6];
    packet_params[0] = (current_config.preamble_length >> 8) & 0xFF;
    packet_params[1] = current_config.preamble_length & 0xFF;
    packet_params[2] = current_config.payload_length == 0 ? 0x00 : 0x01;
    packet_params[3] = len;
    packet_params[4] = current_config.crc_on ? 0x01 : 0x00;
    packet_params[5] = current_config.iq_inverted ? 0x01 : 0x00;

    ret = sx1262_write_command(SX1262_CMD_SET_PACKET_PARAMS, packet_params, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Packet Params Update failed");
        goto cleanup; 
    }

    // Set TX Mode
    uint8_t tx_params[3] = {0x00, 0x00, 0x00}; // No timeout
    ret = sx1262_write_command(SX1262_CMD_SET_TX, tx_params, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setting TX Mode failed");
        goto cleanup; 
    }

    // Wait for TX Done
    uint32_t timeout = 5000;
    uint32_t start = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout)) {
        uint16_t irq_status = sx1262_get_irq_status();
        if (irq_status & SX1262_IRQ_TX_DONE) {
            ret = sx1262_clear_irq_status(SX1262_IRQ_TX_DONE);
            if (ret != ESP_OK) {
                goto cleanup; 
            }

            ESP_LOGD(TAG, "TX Done");
            ret = ESP_OK;
            goto cleanup;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGE(TAG, "TX Timeout");
    
    // Timeout
    ret = ESP_ERR_TIMEOUT;
    
cleanup:
    // ONE place for Cleanup!
    xSemaphoreGiveRecursive(sx1262_mutex);
    return ret;
}

esp_err_t sx1262_receive(uint8_t *data, uint8_t *len, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;

    if (!hw_initialized) {
        ESP_LOGE(TAG, "Hardware not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

     if (data == NULL || len == NULL || !sx1262_mutex) {
        return ESP_ERR_INVALID_ARG;
    }

    // Take mutex (timeout slightly longer than receive timeout)
    if (xSemaphoreTakeRecursive(sx1262_mutex, pdMS_TO_TICKS(timeout_ms + 1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for receive");
        return ESP_ERR_TIMEOUT;
    }

    // Standby Mode
    ret = sx1262_standby();
    if (ret != ESP_OK) {
        goto cleanup;
    }

    // Clear IRQ Status
    sx1262_clear_irq_status(0xFFFF);

    // Set RX Mode
    uint8_t rx_params[3];
    if (timeout_ms == 0) {
        // Continuous RX
        rx_params[0] = 0xFF;
        rx_params[1] = 0xFF;
        rx_params[2] = 0xFF;
    } else {
        // Timeout in 15.625 us steps
        uint32_t timeout_steps = (timeout_ms * 1000) / 15.625;
        rx_params[0] = (timeout_steps >> 16) & 0xFF;
        rx_params[1] = (timeout_steps >> 8) & 0xFF;
        rx_params[2] = timeout_steps & 0xFF;
    }

    ret = sx1262_write_command(SX1262_CMD_SET_RX, rx_params, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setting RX Mode failed");
        goto cleanup;
    }

    // Wait for RX Done
    uint32_t wait_timeout = timeout_ms == 0 ? 60000 : timeout_ms + 1000;
    uint32_t start = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(wait_timeout)) {
        uint16_t irq_status = sx1262_get_irq_status();
        
        if (irq_status & SX1262_IRQ_RX_DONE) {
            sx1262_clear_irq_status(SX1262_IRQ_RX_DONE);
            
            // Read Buffer Status
            uint8_t buffer_status[2];
            ret = sx1262_read_command(SX1262_CMD_GET_RX_BUFFER_STATUS, buffer_status, 2);
            if (ret != ESP_OK) {
                goto cleanup;
            }

            uint8_t payload_len = buffer_status[0];
            uint8_t rx_start_ptr = buffer_status[1];

            if (payload_len > 255) {
                payload_len = 255;
            }

            // Read data from buffer using the general function
            // We need the SPI sequence: [CMD_READ_BUFFER, OFFSET, NOP]
            
            uint8_t tx_header[3];
            tx_header[0] = SX1262_CMD_READ_BUFFER; // The command
            tx_header[1] = rx_start_ptr;          // The offset
            tx_header[2] = 0x00;                  // The NOP
            
            // Read 'payload_len' bytes directly into the 'data' buffer
            // after sending the 3-byte header.
            ret = sx1262_spi_read_general(tx_header, 3, data, payload_len);

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "ReadBuffer failed");
                goto cleanup; 
            }

            // No more memcpy() needed, data was filled directly!
            *len = payload_len;

            ESP_LOGD(TAG, "RX Done: %d bytes", payload_len);
            ret = ESP_OK;
            goto cleanup;

        }
        
        if (irq_status & SX1262_IRQ_TIMEOUT) {
            sx1262_clear_irq_status(SX1262_IRQ_TIMEOUT);
            ESP_LOGD(TAG, "RX Timeout");
            ret = ESP_ERR_TIMEOUT;
            goto cleanup;
        }
        
        if (irq_status & SX1262_IRQ_CRC_ERROR) {
            ret = sx1262_clear_irq_status(SX1262_IRQ_CRC_ERROR);
            if (ret != ESP_OK) {
                goto cleanup; 
            }

            ESP_LOGW(TAG, "CRC Error");
            ret = ESP_FAIL;
            goto cleanup;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ret = ESP_ERR_TIMEOUT;

cleanup:
    xSemaphoreGiveRecursive(sx1262_mutex);
    return ret;
}

// ============================================================================
// HELPER FUNCTIONS 
// ============================================================================

esp_err_t sx1262_sleep(void)
{
    if (!sx1262_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTakeRecursive(sx1262_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for sleep");
        return ESP_ERR_TIMEOUT;
    }

    uint8_t sleep_config = 0x04; // Warm start
    esp_err_t ret = sx1262_write_command(SX1262_CMD_SET_SLEEP, &sleep_config, 1);

    xSemaphoreGiveRecursive(sx1262_mutex);
    
    return ret;
}

esp_err_t sx1262_standby(void)
{
    if (!sx1262_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTakeRecursive(sx1262_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for standby");
        return ESP_ERR_TIMEOUT;
    }

    uint8_t standby_config = 0x01; // STDBY_XOSC
    esp_err_t ret = sx1262_write_command(SX1262_CMD_SET_STANDBY, &standby_config, 1);

    xSemaphoreGiveRecursive(sx1262_mutex);
    
    return ret;   
}

int16_t sx1262_get_rssi(void)
{
    if (!sx1262_mutex) {
        return -999;
    }

    if (xSemaphoreTakeRecursive(sx1262_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for rssi");
        return -999;
    }

    uint8_t rssi_data[1];
    esp_err_t ret = sx1262_read_command(SX1262_CMD_GET_RSSI_INST, rssi_data, 1);
    
    int16_t rssi;
    if (ret != ESP_OK) {
        rssi = -999;
    } else {
        rssi = -(int16_t)(rssi_data[0]) / 2;
    }
    
    xSemaphoreGiveRecursive(sx1262_mutex);
    return rssi;
}

esp_err_t sx1262_get_packet_status(sx1262_packet_status_t *status)
{
    esp_err_t ret = ESP_OK;

    if (status == NULL || !sx1262_mutex) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTakeRecursive(sx1262_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for packet status");
        return ESP_ERR_TIMEOUT;
    }

    uint8_t pkt_status[3];
    ret = sx1262_read_command(SX1262_CMD_GET_PACKET_STATUS, pkt_status, 3);
    
    if (ret != ESP_OK) {
        goto cleanup;

    }

    if (current_config.modem_mode == SX1262_MODEM_LORA) {
        // LoRa Mode
        status->rssi_pkt = -(int16_t)(pkt_status[0]) / 2;
        status->snr_pkt = ((int8_t)pkt_status[1]) * 0.25;
        status->signal_rssi = -(int16_t)(pkt_status[2]) / 2;
    } else {
        // FSK Mode
        status->rssi_pkt = -(int16_t)(pkt_status[0]) / 2;
        status->snr_pkt = 0;
        status->signal_rssi = 0;
    }

cleanup:
    // Release mutex
    xSemaphoreGiveRecursive(sx1262_mutex);
    return ret;
}

esp_err_t sx1262_get_chip_info(void)
{
    esp_err_t ret;

    if (!sx1262_mutex) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Take mutex
    if (xSemaphoreTakeRecursive(sx1262_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex for send");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "=== SX1262 Chip Information ===");
    
    // 1. Get Status
    uint8_t status[1];
    ret = sx1262_read_command(SX1262_CMD_GET_STATUS, status, 1);
    if (ret == ESP_OK) {
        uint8_t chip_mode = (status[0] >> 4) & 0x07;
        uint8_t cmd_status = (status[0] >> 1) & 0x07;
        
        const char* mode_str[] = {"UNUSED", "RFU", "STDBY_RC", "STDBY_XOSC", 
                                  "FS", "RX", "TX", "UNUSED"};
        const char* cmd_str[] = {"UNUSED", "RFU", "Data available", "Timeout",
                                 "Processing error", "Execution failure", "TX done", "UNUSED"};
        
        ESP_LOGI(TAG, "Status: 0x%02X", status[0]);
        ESP_LOGI(TAG, "  Chip Mode: %s", mode_str[chip_mode]);
        ESP_LOGI(TAG, "  Command Status: %s", cmd_str[cmd_status]);
    } else {
        ESP_LOGE(TAG, "Status query failed - Chip not reachable!");
        goto cleanup; 
    }
    
    // 2. Read Packet Type
    uint8_t packet_type[1];
    ret = sx1262_read_command(SX1262_CMD_GET_PACKET_TYPE, packet_type, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Packet Type: %s", 
                 packet_type[0] == 0x00 ? "GFSK" : 
                 packet_type[0] == 0x01 ? "LoRa" : "Unknown");
    }
    
    // 3. Random Number Generator Test (checks if chip is working)
    uint8_t random[4];
    ret = sx1262_read_register(SX1262_REG_RANDOM_NUMBER_GEN, random, 4);
    if (ret == ESP_OK) {
        uint32_t rnd = (random[0] << 24) | (random[1] << 16) | 
                       (random[2] << 8) | random[3];
        ESP_LOGI(TAG, "Random Number: 0x%08lX", rnd);
        
        // If Random = 0 or 0xFFFFFFFF -> Problem
        if (rnd == 0 || rnd == 0xFFFFFFFF) {
            ESP_LOGW(TAG, "  ⚠ Suspicious Random Number - possible chip problem");
        } else {
            ESP_LOGI(TAG, "  ✓ Chip is responding correctly");
        }
    }
    
    // 4. Read Sync Word (LoRa only)
    if (current_config.modem_mode == SX1262_MODEM_LORA) {
        uint8_t sync_msb, sync_lsb;
        ret = sx1262_read_register(SX1262_REG_LORA_SYNC_WORD_MSB, &sync_msb, 1);
        if (ret == ESP_OK) {
            sx1262_read_register(SX1262_REG_LORA_SYNC_WORD_LSB, &sync_lsb, 1);
            uint16_t sync_word = (sync_msb << 8) | sync_lsb;
            ESP_LOGI(TAG, "Sync Word: 0x%04X", sync_word);
            
            if (sync_word == 0x1424) {
                ESP_LOGI(TAG, "  → LoRaWAN Public Network");
            } else if (sync_word == 0x3444) {
                ESP_LOGI(TAG, "  → LoRaWAN Private Network");
            } else {
                ESP_LOGI(TAG, "  → Custom Network");
            }
        }
    }
    
    // 5. Over Current Protection
    uint8_t ocp[1];
    ret = sx1262_read_register(SX1262_REG_OCP_CONFIGURATION, ocp, 1);
    if (ret == ESP_OK) {
        uint8_t ocp_trim = ocp[0] & 0x3F;
        float ocp_ma = ocp_trim * 2.5;
        ESP_LOGI(TAG, "OCP Limit: %.1f mA", ocp_ma);
    }
    
    // 6. RX Gain
    uint8_t rx_gain[1];
    ret = sx1262_read_register(SX1262_REG_RX_GAIN, rx_gain, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RX Gain: 0x%02X (%s)", rx_gain[0],
                 rx_gain[0] == 0x94 ? "Power Saving" :
                 rx_gain[0] == 0x96 ? "Boosted" : "Unknown");
    }
    
    ESP_LOGI(TAG, "=================================\n");
    
cleanup:
    xSemaphoreGiveRecursive(sx1262_mutex);
    return ret;
}

// ============================================================================
// STATIC HELPER FUNCTIONS 
// ============================================================================

static void sx1262_reset(void)
{
    gpio_set_level(LORA_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LORA_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void sx1262_wait_on_busy(void)
{
    uint32_t timeout = 1000;
    uint32_t start = xTaskGetTickCount();
    
    while (gpio_get_level(LORA_PIN_BUSY) == 1) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout)) {
            ESP_LOGW(TAG, "BUSY Timeout");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static esp_err_t sx1262_write_command(uint8_t cmd, uint8_t *data, uint8_t len)
{
    // The header is just the single command byte 'cmd'
    // We pass the address of 'cmd' as a 1-byte header
    return sx1262_spi_write_general(&cmd, 1, data, len);
}

static esp_err_t sx1262_read_command(uint8_t cmd, uint8_t *data, uint8_t len)
{
    // Send [CMD, NOP], read 'len' bytes
    uint8_t tx_header[2];
    tx_header[0] = cmd;
    tx_header[1] = 0x00; // NOP
    
    // Header is 2 bytes long
    return sx1262_spi_read_general(tx_header, 2, data, len);
}


static esp_err_t sx1262_write_register(uint16_t addr, uint8_t *data, uint8_t len)
{
    // The header is [CMD, ADDR_H, ADDR_L]
    uint8_t tx_header[3];
    tx_header[0] = SX1262_CMD_WRITE_REGISTER;
    tx_header[1] = (addr >> 8) & 0xFF;
    tx_header[2] = addr & 0xFF;
    
    // Header is 3 bytes long
    return sx1262_spi_write_general(tx_header, 3, data, len);
}

static esp_err_t sx1262_read_register(uint16_t addr, uint8_t *data, uint8_t len)
{
    // Send [CMD_READ, ADDR_H, ADDR_L, NOP], read 'len' bytes
    uint8_t tx_header[4];
    tx_header[0] = SX1262_CMD_READ_REGISTER;
    tx_header[1] = (addr >> 8) & 0xFF;
    tx_header[2] = addr & 0xFF;
    tx_header[3] = 0x00; // NOP
    
    // Header is 4 bytes long
    return sx1262_spi_read_general(tx_header, 4, data, len);
}


/**
 * @brief General, flexible SPI write function
 * Sends a variable-length header, followed by optional data.
 * This function implements the correct "wait-transmit-wait" cycle.
 *
 * @param tx_header         Buffer with the command/address bytes to be sent
 * @param tx_header_len     Number of bytes in tx_header
 * @param data              Optional data sent after the header
 * @param data_len          Number of optional data bytes
 * @return esp_err_t 
 */
static esp_err_t sx1262_spi_write_general(uint8_t *tx_header, uint8_t tx_header_len, uint8_t *data, uint8_t data_len)
{
    // Wait No. 1: Wait until the chip is ready for a command
    sx1262_wait_on_busy();
    
    // Total transaction size
    uint8_t total_len = tx_header_len + data_len;
    
    // We need a single, contiguous buffer for SPI
    uint8_t tx_buffer[total_len]; 
    
    // 1. Copy header into the buffer
    memcpy(tx_buffer, tx_header, tx_header_len);
    
    // 2. Copy optional data into the buffer
    if (data != NULL && data_len > 0) {
        memcpy(&tx_buffer[tx_header_len], data, data_len);
    }
    
    spi_transaction_t trans = {
        .length = total_len * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = NULL
    };
    
    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    
    // Wait No. 2: Wait until the chip has finished executing the command
    sx1262_wait_on_busy();
    
    return ret;
}



/**
 * @brief General, flexible SPI read function
 * Sends a variable-length header (e.g., [CMD] or [CMD, OFFSET] or [CMD, ADDR_H, ADDR_L])
 * followed by NOPs to receive 'rx_len' bytes.
 *
 * @param tx_header         Buffer with the command/address bytes to be sent
 * @param tx_header_len     Number of bytes in tx_header
 * @param rx_data           Destination buffer for the read data
 * @param rx_len            Number of data bytes to read
 * @return esp_err_t 
 */
static esp_err_t sx1262_spi_read_general(uint8_t *tx_header, uint8_t tx_header_len, uint8_t *rx_data, uint8_t rx_len)
{
    sx1262_wait_on_busy();
    
    // We need a transaction buffer that can hold the header + read part
    // The ESP-IDF SPI drivers require buffers suitable for DMA.
    // A static buffer is often problematic here. It is better to
    // allocate tx_buffer and rx_buffer dynamically on the stack.
    
    uint8_t tx_buffer[tx_header_len + rx_len];
    uint8_t rx_buffer[tx_header_len + rx_len];

    // 1. Copy send header
    memcpy(tx_buffer, tx_header, tx_header_len);
    
    // 2. Fill the rest of the send buffer with NOPs (optional, 0x00 is default)
    // memset(&tx_buffer[tx_header_len], 0x00, rx_len);
    
    spi_transaction_t trans = {
        .length = (tx_header_len + rx_len) * 8, // Total length in bits
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };
    
    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    
    if (ret == ESP_OK && rx_data != NULL && rx_len > 0) {
        // The read data starts *after* the header part
        memcpy(rx_data, &rx_buffer[tx_header_len], rx_len);
    }
    
    return ret;
}


static esp_err_t sx1262_set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask)
{
    uint8_t irq_params[8];
    irq_params[0] = (irq_mask >> 8) & 0xFF;
    irq_params[1] = irq_mask & 0xFF;
    irq_params[2] = (dio1_mask >> 8) & 0xFF;
    irq_params[3] = dio1_mask & 0xFF;
    irq_params[4] = (dio2_mask >> 8) & 0xFF;
    irq_params[5] = dio2_mask & 0xFF;
    irq_params[6] = (dio3_mask >> 8) & 0xFF;
    irq_params[7] = dio3_mask & 0xFF;
    
    return sx1262_write_command(SX1262_CMD_SET_DIO_IRQ_PARAMS, irq_params, 8);
}

static esp_err_t sx1262_clear_irq_status(uint16_t irq_mask)
{
    uint8_t irq_params[2];
    irq_params[0] = (irq_mask >> 8) & 0xFF;
    irq_params[1] = irq_mask & 0xFF;
    
    return sx1262_write_command(SX1262_CMD_CLR_IRQ_STATUS, irq_params, 2);
}

static uint16_t sx1262_get_irq_status(void)
{
    uint8_t irq_status[2];
    esp_err_t ret = sx1262_read_command(SX1262_CMD_GET_IRQ_STATUS, irq_status, 2);
    
    if (ret != ESP_OK) {
        return 0;
    }
    
    return ((uint16_t)irq_status[0] << 8) | irq_status[1];
}