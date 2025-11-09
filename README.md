# LoRa TestTool für Heltec WiFi LoRa 32 V3.2

**Interaktives LoRa-Testtool mit OLED-Display und Echtzeit-Parameterkonfiguration**

![ESP32](https://img.shields.io/badge/ESP32-S3-blue) ![LoRa](https://img.shields.io/badge/LoRa-SX1262-green) ![License](https://img.shields.io/badge/license-MIT-orange)

---

## Features

✅ **Interaktives Menü** auf 128x64 OLED Display  
✅ **Intuitive Navigation** mit Short-Click und Long-Press  
✅ **Live-Konfiguration** aller LoRa-Parameter ohne Neustart  
✅ **Send-Modus** mit Paket-Zähler und Send-Indikator  
✅ **Receive-Modus** mit RSSI-Anzeige und Paket-Statistik  
✅ **Echtzeit-Updates** beim Parameterwechsel  

## Hardware

- **Board**: Heltec WiFi LoRa 32 V3.2
- **Display**: 128x64 OLED (SSD1306)
- **LoRa**: SX1262 Transceiver
- **Button**: GPIO 0 (Boot-Button)

⚠️ **Wichtig:** Immer Antenne anschließen! (868 MHz für EU, 915 MHz für US)

## Display Layout

```
┌────────────────────────────┐
│ >Mode: Send           [*]  │ ← Aktiver Menüpunkt (>)
│  SF: 7                     │   Editier-Modus ([invertiert])
│  BW: 125                   │
│  CR: 4/5                   │
│  Pwr: 14                   │
│ ─────────────────────────  │
│ TX: 42                   * │ ← Status: Pakete + Send-Indikator
└────────────────────────────┘

Im Receive-Modus:
┌────────────────────────────┐
│  Mode: Recv                │
│  SF: 7                     │
│  BW: 125                   │
│  CR: 4/5                   │
│  Pwr: 14                   │
│ ─────────────────────────  │
│ RX:15 RSSI:-85             │ ← Empfangene Pakete + RSSI
└────────────────────────────┘
```

## Navigation

### Linke Seite (Menü-Navigation)

**Short Click (< 500ms)**
- Springt zum nächsten Menüpunkt
- Zyklisch: Mode → SF → BW → CR → Power → Mode

**Long Press (≥ 500ms)**
- Wechselt in den Editier-Modus (rechte Seite)
- Der aktuelle Wert wird invertiert dargestellt

### Rechte Seite (Wert-Editierung)

**Short Click (< 500ms)**
- Wechselt zum nächsten Wert (zyklisch durch Optionen)
- Aktualisiert **sofort** die LoRa-Konfiguration

**Long Press (≥ 500ms)**
- Verlässt den Editier-Modus
- Kehrt zur Menü-Navigation zurück (linke Seite)

## Parameter-Bereiche

| Parameter | Werte | Beschreibung |
|-----------|-------|--------------|
| **Mode** | Send / Receive | Betriebsmodus |
| **SF** | 5 - 12 | Spreading Factor (zyklisch) |
| **BW** | 125 / 250 / 500 | Bandwidth in kHz (zyklisch) |
| **CR** | 4/5, 4/6, 4/7, 4/8 | Coding Rate (zyklisch) |
| **TX Power** | -9 bis +22 | Sendeleistung in dBm (zyklisch) |

## Betriebsmodi

### Send-Modus
- Sendet alle 2 Sekunden ein Paket
- Paketinhalt: `"PKT:42 SF:7 BW:125"` (mit Counter)
- Display zeigt:
  - `TX: 42` → Anzahl gesendeter Pakete
  - `*` → Blinkt beim Senden

### Receive-Modus
- Hört kontinuierlich auf LoRa-Pakete
- Display zeigt:
  - `RX:15` → Anzahl empfangener Pakete
  - `RSSI:-85` → Signalstärke des letzten Pakets
  - `Waiting...` → Wenn noch nichts empfangen

## Projektstruktur


```
lora-testtool/
├── main/
│   ├── main.c              # Hauptprogramm
│   ├── lora_testtool.c     # Testtool-Logik
│   └── lora_testtool.h
├── components/
│   ├── button/             # Button-Handler
│   │   ├── button.c
│   │   └── button.h
│   └── sx1262/             # LoRa-Treiber
│       ├── sx1262.c
│       └── sx1262.h
└── CMakeLists.txt
```

### Dependencies

Das Projekt benötigt:
- ✅ **button** - Button-Library (bereits vorhanden)
- ✅ **sx1262** - Dein LoRa-Treiber (bereits vorhanden)
- 📦 **u8g2** - Display-Library ([u8g2](https://github.com/olikraus/u8g2))
- 📦 **u8g2_hal_esp32** - Display-Library ESP32 HAL([u8g2_hal_esp32](https://github.com/mkfrey/u8g2-hal-esp-idf))


## Technische Details

### Task-Architektur

Das Tool erstellt 3 Tasks:

1. **Button Task** (Priorität 1)
   - Überwacht GPIO 0
   - Debouncing, Double-Click-Erkennung
   - Stack: 2 KB

2. **LoRa Send Task** (Priorität 5)
   - Sendet Pakete im Send-Modus
   - Intervall: 2 Sekunden
   - Stack: 4 KB

3. **LoRa Receive Task** (Priorität 5)
   - Empfängt Pakete im Receive-Modus
   - Non-blocking mit 100ms Timeout
   - Stack: 4 KB

### Latenz

- Display-Update: ~50ms
- Parameter-Änderung: ~100ms (inkl. LoRa-Rekonfiguration)
- Button-Reaktion: 10-420ms (abhängig von Double-Click)

## Anpassungen

### Sendeintervall ändern

In `lora_testtool.c`:
```c
#define SEND_INTERVAL_MS    2000  // Ändere auf gewünschten Wert
```

### Frequenz ändern

In `update_lora_config()`:
```c
.frequency = 868000000,  // Ändere auf deine ISM-Band-Frequenz
```

### Weitere Parameter hinzufügen

1. Enum `menu_item_t` erweitern
2. State `menu_state_t` erweitern
3. `draw_menu_item()` Aufruf in `update_display()` hinzufügen
4. Case in `menu_next_value()` hinzufügen
5. `update_lora_config()` anpassen

## Timing-Diagramm

### Send-Modus Ablauf

```
Zeit: 0s      2s      4s      6s
      ↓       ↓       ↓       ↓
Send: ████    ████    ████    ████
      ↑       ↑       ↑       ↑
      PKT:1   PKT:2   PKT:3   PKT:4

Display:
      TX: 1   TX: 2   TX: 3   TX: 4
      *       *       *       *
```

### Receive-Modus Ablauf

```
Zeit: 0s         RX        RX           RX
      ↓          ↓         ↓            ↓
Recv: ─────────███────────███─────────███
      ↑         ↑         ↑            ↑
      Waiting   RSSI:-85  RSSI:-82     RSSI:-90

Display:
      Waiting   RX:1      RX:2         RX:3
                RSSI:-85  RSSI:-82     RSSI:-90
```
## 🔗 Links

- [ESP-IDF Dokumentation](https://docs.espressif.com/projects/esp-idf/)
- [SX1262 Datenblatt](https://www.semtech.com/products/wireless-rf/lora-core/sx1262)
- [LoRa Grundlagen](https://www.semtech.com/lora/resources/lora-community/)