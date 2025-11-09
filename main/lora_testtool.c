/**
 * @file lora_testtool.c
 * @brief LoRa Test Tool mit interaktivem Menü für Heltec WiFi LoRa 32 V3.2
 * 
 * Navigation:
 * Linke Seite (Menü):
 *   - Short Click: Nächster Menüpunkt
 *   - Long Press: Wechsel zu rechter Seite (Edit-Modus)
 * Rechte Seite (Edit):
 *   - Short Click: Nächster Wert
 *   - Long Press: Zurück zu linker Seite (Menü)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "u8g2.h"
#include "button.h"
#include "sx1262.h"

static const char* TAG = "LORA_TOOL";

// ============================================================================
// CONFIGURATION
// ============================================================================

#define BUTTON_PIN          GPIO_NUM_0
#define LED_PIN             GPIO_NUM_35
#define SEND_INTERVAL_MS    2000
#define PACKET_SIZE         32

// ============================================================================
// MENU STRUCTURE
// ============================================================================

typedef enum {
    MENU_MODE = 0,
    MENU_SF,
    MENU_BW,
    MENU_CR,
    MENU_TX_POWER,
    MENU_COUNT  // Anzahl der Menüpunkte
} menu_item_t;

typedef enum {
    MODE_SEND = 0,
    MODE_RECEIVE
} operation_mode_t;

typedef struct {
    menu_item_t current_item;           // Aktueller Menüpunkt
    bool editing;                       // true = Wert editieren, false = Menü navigieren
    
    // LoRa Parameter
    operation_mode_t mode;              // Send/Receive
    uint8_t sf;                         // Spreading Factor 5-12
    uint16_t bw;                        // Bandwidth 125/250/500
    sx1262_coding_rate_t cr;            // Coding Rate 4/5, 4/6, 4/7, 4/8
    int8_t tx_power;                    // TX Power -9 bis 22
    
    // Status
    bool is_sending;                    // Wird gerade gesendet?
    int16_t last_rssi;                  // Letzter RSSI-Wert
    uint32_t packets_sent;              // Anzahl gesendeter Pakete
    uint32_t packets_received;          // Anzahl empfangener Pakete
    uint32_t last_packet_time;          // Zeitstempel des letzten Pakets
} menu_state_t;

// Globale Variablen
static menu_state_t menu;
static u8g2_t u8g2;
static button_handle_t* button;
static TaskHandle_t lora_task_handle = NULL;
static SemaphoreHandle_t display_mutex = NULL;
static volatile bool display_needs_update = false;

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

/**
 * @brief Konvertiert Bandwidth-Wert zu String
 */
static const char* bw_to_string(uint16_t bw) {
    switch(bw) {
        case 125: return "125";
        case 250: return "250";
        case 500: return "500";
        default: return "???";
    }
}

/**
 * @brief Konvertiert Coding Rate zu String
 */
static const char* cr_to_string(sx1262_coding_rate_t cr) {
    switch(cr) {
        case LORA_CR_4_5: return "4/5";
        case LORA_CR_4_6: return "4/6";
        case LORA_CR_4_7: return "4/7";
        case LORA_CR_4_8: return "4/8";
        default: return "???";
    }
}

/**
 * @brief Konvertiert Bandwidth zu SX1262 enum
 */
static sx1262_bandwidth_t bw_to_enum(uint16_t bw) {
    switch(bw) {
        case 125: return LORA_BW_125;
        case 250: return LORA_BW_250;
        case 500: return LORA_BW_500;
        default: return LORA_BW_125;
    }
}

/**
 * @brief Zeichnet einen Menüpunkt
 */
static void draw_menu_item(int y, const char* label, const char* value, bool is_active, bool is_editing) {
    // Cursor für aktiven Menüpunkt
    if (is_active && !is_editing) {
        u8g2_DrawStr(&u8g2, 0, y, ">");
    }
    
    // Label
    u8g2_DrawStr(&u8g2, 8, y, label);
    
    // Wert (rechts ausgerichtet, invertiert wenn editing)
    int value_width = u8g2_GetStrWidth(&u8g2, value);
    int value_x = 128 - value_width - 2;
    
    if (is_active && is_editing) {
        // Invertierter Balken für Edit-Modus
        u8g2_SetDrawColor(&u8g2, 1);
        u8g2_DrawBox(&u8g2, value_x - 2, y - 10, value_width + 4, 12);
        u8g2_SetDrawColor(&u8g2, 0);
        u8g2_DrawStr(&u8g2, value_x, y, value);
        u8g2_SetDrawColor(&u8g2, 1);
    } else {
        u8g2_DrawStr(&u8g2, value_x, y, value);
    }
}

/**
 * @brief Zeichnet die Statuszeile
 */
static void draw_status_line(void) {
    char status[32];
    
    // Trennlinie
    u8g2_DrawHLine(&u8g2, 0, 52, 128);
    
    if (menu.mode == MODE_SEND) {
        if (menu.is_sending) {
            snprintf(status, sizeof(status), "TX: %lu", menu.packets_sent);
            // Blink-Indikator
            if ((xTaskGetTickCount() / 100) % 2 == 0) {
                u8g2_DrawStr(&u8g2, 100, 60, "*");
            }
        } else {
            snprintf(status, sizeof(status), "Sent: %lu", menu.packets_sent);
        }
    } else {
        if (menu.packets_received > 0) {
            snprintf(status, sizeof(status), "RX:%ld RSSI:%d", 
                     menu.packets_received, menu.last_rssi);
        } else {
            snprintf(status, sizeof(status), "Waiting...");
        }
    }
    
    u8g2_DrawStr(&u8g2, 2, 62, status);
}

/**
 * @brief Hauptfunktion zum Zeichnen des Displays (thread-safe)
 */
static void update_display(void) {
    // Mutex nehmen (max 100ms warten)
    if (xSemaphoreTake(display_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Could not take display mutex");
        return;
    }
    
    char value_str[16];
    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
    
    // Menüpunkte zeichnen
    // Mode
    snprintf(value_str, sizeof(value_str), "%s", 
             menu.mode == MODE_SEND ? "Send" : "Recv");
    draw_menu_item(10, "Mode:", value_str, 
                   menu.current_item == MENU_MODE, menu.editing);
    
    // SF
    snprintf(value_str, sizeof(value_str), "%d", menu.sf);
    draw_menu_item(20, "SF:", value_str, 
                   menu.current_item == MENU_SF, menu.editing);
    
    // BW
    snprintf(value_str, sizeof(value_str), "%s", bw_to_string(menu.bw));
    draw_menu_item(30, "BW:", value_str, 
                   menu.current_item == MENU_BW, menu.editing);
    
    // CR
    snprintf(value_str, sizeof(value_str), "%s", cr_to_string(menu.cr));
    draw_menu_item(40, "CR:", value_str, 
                   menu.current_item == MENU_CR, menu.editing);
    
    // TX Power
    snprintf(value_str, sizeof(value_str), "%d", menu.tx_power);
    draw_menu_item(50, "Pwr:", value_str, 
                   menu.current_item == MENU_TX_POWER, menu.editing);
    
    // Statuszeile
    draw_status_line();
    
    u8g2_SendBuffer(&u8g2);
    
    // Mutex freigeben
    xSemaphoreGive(display_mutex);
}

/**
 * @brief Markiert Display als "needs update" - wird asynchron aktualisiert
 */
static void request_display_update(void) {
    display_needs_update = true;
}

// ============================================================================
// LORA FUNCTIONS
// ============================================================================

/**
 * @brief Aktualisiert die LoRa-Konfiguration basierend auf Menü-Einstellungen (thread-safe)
 */
static void update_lora_config(void) {
    
    sx1262_config_t config = {
        .modem_mode = SX1262_MODEM_LORA,
        .frequency = 868000000,  // 868 MHz
        .tx_power = menu.tx_power,
        .bandwidth = bw_to_enum(menu.bw),
        .spreading_factor = menu.sf,
        .coding_rate = menu.cr,
        .iq_inverted = false,
        .rx_gain_boosted = true,
        .preamble_length = 8,
        .payload_length = 0,  // Variable length
        .crc_on = true,
        .sync_word = 0x1424  // Public network
    };
    
    esp_err_t err = sx1262_configure(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure SX1262: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "LoRa configured: SF%d BW%d CR%s Power%d", 
                 menu.sf, menu.bw, cr_to_string(menu.cr), menu.tx_power);
    }
    
}

/**
 * @brief LoRa Send Task
 */
static void lora_send_task(void* parameter) {
    uint8_t packet[PACKET_SIZE];
    
    while (true) {
        if (menu.mode == MODE_SEND) {
            // Paket vorbereiten
            snprintf((char*)packet, PACKET_SIZE, 
                     "PKT:%lu SF:%d BW:%d", menu.packets_sent, menu.sf, menu.bw);
            
            // Senden (mit Mutex-Schutz)
            menu.is_sending = true;
            request_display_update();
        
            esp_err_t err = sx1262_send(packet, strlen((char*)packet));
            
            if (err == ESP_OK) {
                menu.packets_sent++;
                ESP_LOGI(TAG, "Sent packet #%lu", menu.packets_sent);
            } else {
                ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(err));
            }
            
            menu.is_sending = false;
            request_display_update();
            
            vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/**
 * @brief LoRa Receive Task
 */
static void lora_receive_task(void* parameter) {
    uint8_t packet[256];
    uint8_t len;
    
    while (true) {
        if (menu.mode == MODE_RECEIVE) {
            esp_err_t err = sx1262_receive(packet, &len, 100);
            
            if (err == ESP_OK && len > 0) {
                menu.packets_received++;

                // LED einschalten
                gpio_set_level(LED_PIN, 1);  // AN

                menu.last_packet_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                
                // RSSI auslesen
                sx1262_packet_status_t status;
                if (sx1262_get_packet_status(&status) == ESP_OK) {
                    menu.last_rssi = status.rssi_pkt;
                    ESP_LOGI(TAG, "RX #%lu: %d bytes, RSSI:%d, SNR:%.1f", 
                                menu.packets_received, len, 
                                status.rssi_pkt, status.snr_pkt);
                }
                
                // Display aktualisieren
                request_display_update();
                
                // Paket ausgeben
                packet[len] = '\0';
                ESP_LOGI(TAG, "Data: %s", packet);

                // LED ausschalten nach 100ms
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(LED_PIN, 0);
            }      
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Display Update Task - aktualisiert Display wenn benötigt
 */
static void display_update_task(void* parameter) {
    while (true) {
        // Prüfe ob Update benötigt wird
        if (display_needs_update) {
            display_needs_update = false;
            update_display();
        }
        
        // Regelmäßiges Update für Blink-Indikator im Send-Modus
        if (menu.mode == MODE_SEND && menu.is_sending) {
            update_display();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz Update-Rate
    }
}

// ============================================================================
// MENU NAVIGATION
// ============================================================================

/**
 * @brief Nächster Menüpunkt (zyklisch)
 */
static void menu_next_item(void) {
    menu.current_item = (menu.current_item + 1) % MENU_COUNT;
    ESP_LOGI(TAG, "Menu item: %d", menu.current_item);
    request_display_update();
}

/**
 * @brief Nächster Wert für aktuellen Menüpunkt
 */
static void menu_next_value(void) {
    bool config_changed = true;
    
    switch(menu.current_item) {
        case MENU_MODE:
            menu.mode = (menu.mode == MODE_SEND) ? MODE_RECEIVE : MODE_SEND;
            // Reset counters beim Modewechsel
            menu.packets_sent = 0;
            menu.packets_received = 0;
            menu.last_rssi = 0;
            ESP_LOGI(TAG, "Mode: %s", menu.mode == MODE_SEND ? "SEND" : "RECEIVE");
            break;
            
        case MENU_SF:
            menu.sf++;
            if (menu.sf > 12) menu.sf = 5;
            ESP_LOGI(TAG, "SF: %d", menu.sf);
            break;
            
        case MENU_BW:
            if (menu.bw == 125) menu.bw = 250;
            else if (menu.bw == 250) menu.bw = 500;
            else menu.bw = 125;
            ESP_LOGI(TAG, "BW: %d", menu.bw);
            break;
            
        case MENU_CR:
            // Cycle through coding rates: 4/5 -> 4/6 -> 4/7 -> 4/8 -> 4/5
            switch(menu.cr) {
                case LORA_CR_4_5: menu.cr = LORA_CR_4_6; break;
                case LORA_CR_4_6: menu.cr = LORA_CR_4_7; break;
                case LORA_CR_4_7: menu.cr = LORA_CR_4_8; break;
                case LORA_CR_4_8: menu.cr = LORA_CR_4_5; break;
                default: menu.cr = LORA_CR_4_5; break;
            }
            ESP_LOGI(TAG, "CR: %s", cr_to_string(menu.cr));
            break;
            
        case MENU_TX_POWER:
            menu.tx_power++;
            if (menu.tx_power > 22) menu.tx_power = -9;
            ESP_LOGI(TAG, "TX Power: %d dBm", menu.tx_power);
            break;
            
        default:
            config_changed = false;
            break;
    }
    
    // LoRa-Konfiguration aktualisieren
    if (config_changed) {
        update_lora_config();
    }
    
    request_display_update();
}

/**
 * @brief Wechsel in Edit-Modus
 */
static void menu_enter_edit_mode(void) {
    menu.editing = true;
    ESP_LOGI(TAG, "Edit mode: ON");
    request_display_update();
}

/**
 * @brief Wechsel zurück in Menü-Modus
 */
static void menu_exit_edit_mode(void) {
    menu.editing = false;
    ESP_LOGI(TAG, "Edit mode: OFF");
    request_display_update();
}

// ============================================================================
// BUTTON CALLBACKS
// ============================================================================

/**
 * @brief Button Callback - behandelt alle Button-Events
 * 
 * NAVIGATION:
 * Linke Seite (Menü):
 *   - Short Click: Nächster Menüpunkt
 *   - Long Press: Wechsel zu rechter Seite (Edit-Modus)
 * 
 * Rechte Seite (Edit):
 *   - Short Click: Nächster Wert
 *   - Long Press: Zurück zu linker Seite (Menü-Modus)
 */
static void button_callback(button_press_type_t type) {
    switch(type) {
        case BUTTON_PRESS_SHORT:
            if (menu.editing) {
                // Im Edit-Modus: Nächster Wert
                menu_next_value();
            } else {
                // Im Menü-Modus: Nächster Menüpunkt
                menu_next_item();
            }
            break;
            
        case BUTTON_PRESS_LONG:
            if (menu.editing) {
                // Im Edit-Modus: Zurück zum Menü
                menu_exit_edit_mode();
            } else {
                // Im Menü-Modus: In Edit-Modus wechseln
                menu_enter_edit_mode();
            }
            break;
            
        case BUTTON_PRESS_DOUBLE:
            // Double-Click wird nicht verwendet
            break;
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * @brief Initialisiert das Menü mit Standardwerten
 */
static void init_menu(void) {
    memset(&menu, 0, sizeof(menu_state_t));
    
    menu.current_item = MENU_MODE;
    menu.editing = false;
    menu.mode = MODE_SEND;
    menu.sf = 5;
    menu.bw = 125;
    menu.cr = LORA_CR_4_5;
    menu.tx_power = -9;
    menu.is_sending = false;
    menu.last_rssi = 0;
    menu.packets_sent = 0;
    menu.packets_received = 0;
}

/**
 * @brief Initialisiert den Button
 */
static esp_err_t init_button(void) {
    button_config_t button_config = {
        .gpio_num = BUTTON_PIN,
        .active_low = true,
        .short_press_callback = button_callback,
        .long_press_callback = button_callback,
        .double_click_callback = NULL,  // Wird nicht verwendet
        .enable_repeat = false
    };
    
    button = button_create(&button_config);
    if (!button) {
        ESP_LOGE(TAG, "Failed to create button");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Button initialized on GPIO %d", BUTTON_PIN);
    return ESP_OK;
}

/**
 * @brief Initialisiert das Display
 */
static esp_err_t init_display(u8g2_t* display) {
    if (!display) {
        ESP_LOGE(TAG, "Display pointer is NULL");
        return ESP_FAIL;
    }
    
    // Display-Zeiger übernehmen
    memcpy(&u8g2, display, sizeof(u8g2_t));
    
    // Mutex erstellen für thread-safe Display-Zugriff
    display_mutex = xSemaphoreCreateMutex();
    if (display_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create display mutex");
        return ESP_FAIL;
    }
    
    // Test-Ausgabe
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
    u8g2_DrawStr(&u8g2, 10, 30, "LoRa TestTool");
    u8g2_DrawStr(&u8g2, 20, 45, "Initializing...");
    u8g2_SendBuffer(&u8g2);
    
    ESP_LOGI(TAG, "Display initialized");
    return ESP_OK;
}

/**
 * @brief Startet die LoRa-Tasks und Display-Task
 */
static esp_err_t start_lora_tasks(void) {
    BaseType_t ret;
    
    // Display Update Task (höchste Priorität für UI-Responsiveness)
    ret = xTaskCreate(
        display_update_task,
        "display_upd",
        3072,
        NULL,
        6,  // Höhere Priorität
        NULL
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display update task");
        return ESP_FAIL;
    }
    
    // Send Task
    ret = xTaskCreate(
        lora_send_task,
        "lora_send",
        4096,
        NULL,
        5,
        &lora_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create send task");
        return ESP_FAIL;
    }
    
    // Receive Task
    ret = xTaskCreate(
        lora_receive_task,
        "lora_recv",
        4096,
        NULL,
        5,
        NULL
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create receive task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "LoRa tasks started");
    return ESP_OK;
}

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * @brief Hauptinitialisierung des LoRa TestTools
 * 
 * @param display Zeiger auf initialisiertes u8g2 Display
 * @return ESP_OK bei Erfolg
 */
esp_err_t lora_testtool_init(u8g2_t* display) {
    ESP_LOGI(TAG, "=== LoRa TestTool Initializing ===");
    
    // LED GPIO konfigurieren
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_PIN, 0);  // LED aus

    // Menü initialisieren
    init_menu();
    
    // Display initialisieren
    if (init_display(display) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // SX1262 initialisieren
    ESP_LOGI(TAG, "Initializing SX1262...");
    if (sx1262_init() != ESP_OK) {
        ESP_LOGE(TAG, "SX1262 initialization failed");
        return ESP_FAIL;
    }
    
    // Initiale LoRa-Konfiguration
    update_lora_config();
    
    // Button initialisieren
    if (init_button() != ESP_OK) {
        return ESP_FAIL;
    }
    
    // LoRa-Tasks starten
    if (start_lora_tasks() != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Initiales Display-Update
    vTaskDelay(pdMS_TO_TICKS(1000));  // Kurze Pause für "Initializing..."
    request_display_update();
    
    ESP_LOGI(TAG, "=== LoRa TestTool Ready ===");
    ESP_LOGI(TAG, "Navigation:");
    ESP_LOGI(TAG, "  Left Side (Menu):");
    ESP_LOGI(TAG, "    - Short Click: Next menu item");
    ESP_LOGI(TAG, "    - Long Press:  Enter edit mode");
    ESP_LOGI(TAG, "  Right Side (Edit):");
    ESP_LOGI(TAG, "    - Short Click: Next value");
    ESP_LOGI(TAG, "    - Long Press:  Back to menu");
    
    return ESP_OK;
}

/**
 * @brief Gibt Ressourcen frei
 */
void lora_testtool_deinit(void) {
    if (button) {
        button_delete(button);
    }
    
    if (lora_task_handle) {
        vTaskDelete(lora_task_handle);
    }
}
