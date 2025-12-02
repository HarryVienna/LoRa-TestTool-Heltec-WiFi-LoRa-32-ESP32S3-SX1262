/**
 * @file main.c
 * @brief Example for using the LoRa TestTool
 * 
 * This example shows how to use the LoRa TestTool in an ESP-IDF project.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"

// U8g2 and Hardware
#include "u8g2_esp32_hal.h"
#include "lora_testtool.h"
#include "sx1262.h"

static const char* TAG = "MAIN";

// Display Pins for Heltec WiFi LoRa 32 V3.x & V4
#define PIN_SDA      17
#define PIN_SCL      18
#define PIN_RST      21
#define PIN_VEXT     36


// U8g2 Display Handle
static u8g2_t u8g2;

/**
 * @brief Initializes the OLED Display
 */
static esp_err_t init_display(void) {
     // Configure and enable VExt Pin
     // LOW = Display/LoRa power supply ON
     gpio_config_t io_conf = {
         .pin_bit_mask = (1ULL << PIN_VEXT),
         .mode = GPIO_MODE_OUTPUT,
         .pull_up_en = GPIO_PULLUP_DISABLE,
         .pull_down_en = GPIO_PULLDOWN_DISABLE,
         .intr_type = GPIO_INTR_DISABLE
     };
     gpio_config(&io_conf);
     gpio_set_level(PIN_VEXT, 0);   // LOW = Turn on display
     
     ESP_LOGI(TAG, "VExt activated (Display power supply)");
     
     // Longer delay for stabilization after VExt activation
     vTaskDelay(pdMS_TO_TICKS(200));
     
     // Configure U8g2 ESP32 HAL
     ESP_LOGI(TAG, "Configuring U8g2 HAL...");
     u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
     u8g2_esp32_hal.bus.i2c.sda = PIN_SDA;
     u8g2_esp32_hal.bus.i2c.scl = PIN_SCL;
     u8g2_esp32_hal.reset = PIN_RST;
     u8g2_esp32_hal_init(u8g2_esp32_hal);
     
     ESP_LOGI(TAG, "U8g2 HAL initialized");
     
     // Short pause after I2C Init
     vTaskDelay(pdMS_TO_TICKS(100));
     
     // Initialize display (SSD1306 128x64 OLED)
     ESP_LOGI(TAG, "Setup U8g2 Display Structure...");
     u8g2_Setup_ssd1306_i2c_128x64_noname_f(
         &u8g2,
         U8G2_R0,
         u8g2_esp32_i2c_byte_cb,
         u8g2_esp32_gpio_and_delay_cb
     );
     
     // Set I2C address
     // Heltec V3 typically uses 0x3C
     u8x8_SetI2CAddress(&u8g2.u8x8, 0x3C << 1 );
     
     ESP_LOGI(TAG, "Initializing Display (I2C Address 0x3C/0x78)...");
     
     // Initialize display
     u8g2_InitDisplay(&u8g2);
     u8g2_SetPowerSave(&u8g2, 0); // Turn on display
     
     ESP_LOGI(TAG, "Display initialized");
     return ESP_OK;
}

void my_lora_callback(uint8_t *data, uint8_t len, sx1262_packet_status_t *status) {
    ESP_LOGI("APP", "Received packet: %d bytes, RSSI: %d, SNR: %.2f", 
             len, status->rssi_pkt, status->snr_pkt);
    
    // Daten als String ausgeben (falls Text)
    // printf("Data: %.*s\n", len, data);
}


RTC_DATA_ATTR bool sx1262_is_configured = false;

/**
 * @brief Main function
 */
void app_main(void) {

/*    
    // 1. Bus-Ebene IMMER initialisieren
    // Der ESP32 hat seine GPIO/SPI-Config vergessen, daher muss das immer passieren.
    // Wichtig: Diese Funktion fasst den RESET-Pin NICHT an!
    sx1262_init_bus();

    bool radio_ready = false;

    // 2. Entscheidungsbaum: Woher kommen wir?
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER && sx1262_is_configured) {
        
        ESP_LOGI("APP", "Aufgewacht aus Deep Sleep. Versuche Warmstart...");
        
        // Versuch: SX1262 sanft wecken (NSS Toggle)
        if (sx1262_wakeup() == ESP_OK) {
            ESP_LOGI("APP", "Warmstart erfolgreich! Konfiguration erhalten.");
            radio_ready = true;
        } else {
            ESP_LOGW("APP", "Warmstart fehlgeschlagen (SX1262 hat Amnesie).");
            // radio_ready bleibt false -> Fallback zu Kaltstart
        }
    }

    // 3. Fallback / Kaltstart Logik
    if (!radio_ready) {
        ESP_LOGI("APP", "Führe Kaltstart durch (Hard-Reset & Kalibrierung)...");
        
        // Der teure, langsame Teil:
        sx1262_init_radio(); 
        
        // Konfiguration neu senden
        sx1262_config_t cfg = { 
            .modem_mode = SX1262_MODEM_LORA,
            .frequency = 869525000,
            .tx_power = 22,
            .spreading_factor = 7,
            .bandwidth = LORA_BW_125,
            .coding_rate = LORA_CR_4_5,
            .preamble_length = 8,
            .crc_on = true
        };
        sx1262_configure(&cfg);
        
        // MERKEN für das nächste Mal!
        sx1262_is_configured = true;
    }

    // --- Ab hier ist der Chip bereit ---

    // 4. Aktion: Senden
    const char msg[] = "Deep Sleep Ping";
    sx1262_send((uint8_t*)msg, sizeof(msg));

    // 5. Wieder schlafen legen
    // WICHTIG: sx1262_sleep sendet OpCode 0x84 mit Parameter 0x04 (Retention).
    // Nur so behält der Chip sein Gedächtnis für den nächsten Warmstart!
    sx1262_sleep(); 

    // 6. ESP32 schlafen legen (z.B. 60 Sekunden)
    ESP_LOGI("APP", "Gehe in Deep Sleep...");
    esp_sleep_enable_timer_wakeup(5 * 1000000);
    //esp_deep_sleep_start();


    while (1) {
        // Your main application code runs here
        // Packets are received automatically via interrupt
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Still running...");
    }





    esp_err_t ret;
    
    // Initialize SPI
    ret = sx1262_init_bus();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI init failed!");
        return;
    }

    // Initialize radio
    ret = sx1262_init_radio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Radio init failed!");
        return;
    }
    
    // Configure LoRa parameters
    sx1262_config_t config = {
        .modem_mode = SX1262_MODEM_LORA,
        .frequency = 869525000,  // 868.1 MHz
        .tx_power = 22,          // 22 dBm
        
        // LoRa parameters
        .bandwidth = LORA_BW_125,
        .spreading_factor = 7,
        .coding_rate = LORA_CR_4_5,
        .iq_inverted = false,
        .rx_gain_boosted = false,
        
        // Common parameters
        .preamble_length = 8,
        .payload_length = 0,     // Variable length
        .crc_on = true,
        .sync_word = 0x1424      // Public network
    };
    
    ret = sx1262_configure(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configuration failed!");
        return;
    }
    
    // Starten!
    sx1262_start_receive_async(my_lora_callback);

    while (1) {
        // Your main application code runs here
        // Packets are received automatically via interrupt
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Still running...");
    }


*/

    
     ESP_LOGI(TAG, "=====================================================");
     ESP_LOGI(TAG, "   LoRa TestTool for Heltec WiFi LoRa 32 V3.x & V4   ");
     ESP_LOGI(TAG, "=====================================================");

     
     // Initialize display
     if (init_display() != ESP_OK) {
         ESP_LOGE(TAG, "Display initialization failed!");
         return;
     }
     
     // Show welcome message
     u8g2_ClearBuffer(&u8g2);
     u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
     u8g2_DrawStr(&u8g2, 5, 20, "LoRa TestTool");
     u8g2_DrawStr(&u8g2, 5, 35, "Heltec WiFi LoRa 32");
     u8g2_DrawStr(&u8g2, 5, 50, "Starting...");
     u8g2_SendBuffer(&u8g2);
     
     vTaskDelay(pdMS_TO_TICKS(2000));
     
     // Initialize and start LoRa TestTool
     if (lora_testtool_init(&u8g2) != ESP_OK) {
         ESP_LOGE(TAG, "LoRa TestTool initialization failed!");
         
         // Error message on display
         u8g2_ClearBuffer(&u8g2);
         u8g2_DrawStr(&u8g2, 5, 30, "ERROR:");
         u8g2_DrawStr(&u8g2, 5, 45, "Init failed!");
         u8g2_SendBuffer(&u8g2);
         
         return;
     }
     
     ESP_LOGI(TAG, "LoRa TestTool is running!");
     
     // Main loop - The TestTool now runs independently
     while (1) {
         vTaskDelay(pdMS_TO_TICKS(1000));
     }
}