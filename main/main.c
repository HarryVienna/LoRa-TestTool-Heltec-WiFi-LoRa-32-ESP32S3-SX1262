/**
 * @file main.c
 * @brief Beispiel für die Verwendung des LoRa TestTools
 * 
 * Dieses Beispiel zeigt, wie man das LoRa TestTool in einem ESP-IDF Projekt verwendet.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// U8g2 und Hardware
#include "u8g2_esp32_hal.h"
#include "lora_testtool.h"

static const char* TAG = "MAIN";

// Display Pins für Heltec WiFi LoRa 32 V3.2
#define PIN_SDA     17
#define PIN_SCL     18
#define PIN_RST     21
#define PIN_VEXT    36

// U8g2 Display Handle
static u8g2_t u8g2;

/**
 * @brief Initialisiert das OLED Display
 */
static esp_err_t init_display(void) {
    // VExt Pin konfigurieren und aktivieren
    // LOW = Display/LoRa Stromversorgung EIN
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_VEXT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_VEXT, 0);  // LOW = Display einschalten
    
    ESP_LOGI(TAG, "VExt aktiviert (Display-Stromversorgung)");
    
    // Längere Verzögerung für Stabilisierung nach VExt-Aktivierung
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // U8g2 ESP32 HAL konfigurieren
    ESP_LOGI(TAG, "Konfiguriere U8g2 HAL...");
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.bus.i2c.sda = PIN_SDA;
    u8g2_esp32_hal.bus.i2c.scl = PIN_SCL;
    u8g2_esp32_hal.reset = PIN_RST;
    u8g2_esp32_hal_init(u8g2_esp32_hal);
    
    ESP_LOGI(TAG, "U8g2 HAL initialisiert");
    
    // Kurze Pause nach I2C Init
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Display initialisieren (SSD1306 128x64 OLED)
    ESP_LOGI(TAG, "Setup U8g2 Display Struktur...");
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb
    );
    
    // I2C Adresse setzen
    // Heltec V3 verwendet typischerweise 0x3C
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x3C << 1 );
    
    ESP_LOGI(TAG, "Initialisiere Display (I2C Adresse 0x3C/0x78)...");
    
    // Display initialisieren
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); // Display einschalten
    
    ESP_LOGI(TAG, "Display initialized");
    return ESP_OK;
}

/**
 * @brief Hauptfunktion
 */
void app_main(void) {
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "  LoRa TestTool für Heltec V3.2");
    ESP_LOGI(TAG, "=================================");

    
    // Display initialisieren
    if (init_display() != ESP_OK) {
        ESP_LOGE(TAG, "Display initialization failed!");
        return;
    }
    
    // Willkommensnachricht anzeigen
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
    u8g2_DrawStr(&u8g2, 5, 20, "LoRa TestTool");
    u8g2_DrawStr(&u8g2, 5, 35, "Heltec WiFi LoRa 32");
    u8g2_DrawStr(&u8g2, 5, 50, "Starting...");
    u8g2_SendBuffer(&u8g2);
    
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // LoRa TestTool initialisieren und starten
    if (lora_testtool_init(&u8g2) != ESP_OK) {
        ESP_LOGE(TAG, "LoRa TestTool initialization failed!");
        
        // Fehlermeldung auf Display
        u8g2_ClearBuffer(&u8g2);
        u8g2_DrawStr(&u8g2, 5, 30, "ERROR:");
        u8g2_DrawStr(&u8g2, 5, 45, "Init failed!");
        u8g2_SendBuffer(&u8g2);
        
        return;
    }
    
    ESP_LOGI(TAG, "LoRa TestTool is running!");
    
    // Hauptloop - Das TestTool läuft jetzt selbstständig
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
