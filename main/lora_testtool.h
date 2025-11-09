/**
 * @file lora_testtool.h
 * @brief LoRa Test Tool Header
 */

#ifndef LORA_TESTTOOL_H
#define LORA_TESTTOOL_H

#include "esp_err.h"
#include "u8g2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialisiert das LoRa TestTool
 * 
 * Voraussetzungen:
 * - U8g2 Display muss bereits initialisiert sein
 * - SPI muss noch NICHT initialisiert sein (wird intern gemacht)
 * 
 * @param display Zeiger auf initialisiertes u8g2 Display
 * @return ESP_OK bei Erfolg, ESP_FAIL bei Fehler
 * 
 * @note Nach erfolgreicher Initialisierung läuft das Tool selbstständig
 */
esp_err_t lora_testtool_init(u8g2_t* display);

/**
 * @brief Gibt alle Ressourcen frei
 */
void lora_testtool_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // LORA_TESTTOOL_H
