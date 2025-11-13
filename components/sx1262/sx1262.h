#ifndef SX1262_H
#define SX1262_H

#include "esp_err.h"
#include "driver/spi_master.h"
#include <stdint.h>
#include <stdbool.h>

// Pin definitions for Heltec WiFi LoRa 32 V3.2
#define LORA_PIN_MISO       11
#define LORA_PIN_MOSI       10
#define LORA_PIN_SCK        9
#define LORA_PIN_NSS        8
#define LORA_PIN_RST        12
#define LORA_PIN_BUSY       13
#define LORA_PIN_DIO1       14

// SX1262 Commands
#define SX1262_CMD_SET_SLEEP                0x84
#define SX1262_CMD_SET_STANDBY              0x80
#define SX1262_CMD_SET_FS                   0xC1
#define SX1262_CMD_SET_TX                   0x83
#define SX1262_CMD_SET_RX                   0x82
#define SX1262_CMD_STOP_TIMER_ON_PREAMBLE   0x9F
#define SX1262_CMD_SET_RX_DUTY_CYCLE        0x94
#define SX1262_CMD_SET_CAD                 0xC5
#define SX1262_CMD_SET_TX_CONTINUOUS_WAVE   0xD1
#define SX1262_CMD_SET_TX_INFINITE_PREAMBLE 0xD2
#define SX1262_CMD_SET_REGULATOR_MODE       0x96
#define SX1262_CMD_CALIBRATE                0x89
#define SX1262_CMD_CALIBRATE_IMAGE          0x98
#define SX1262_CMD_SET_PA_CONFIG            0x95
#define SX1262_CMD_SET_RX_TX_FALLBACK_MODE   0x93

// Configuration commands
#define SX1262_CMD_WRITE_REGISTER           0x0D
#define SX1262_CMD_READ_REGISTER            0x1D
#define SX1262_CMD_WRITE_BUFFER             0x0E
#define SX1262_CMD_READ_BUFFER              0x1E

// Communication commands
#define SX1262_CMD_GET_STATUS               0xC0
#define SX1262_CMD_GET_RX_BUFFER_STATUS     0x13
#define SX1262_CMD_GET_PACKET_STATUS        0x14
#define SX1262_CMD_GET_RSSI_INST            0x15
#define SX1262_CMD_GET_STATS                0x10
#define SX1262_CMD_RESET_STATS              0x00

// DIO and IRQ commands
#define SX1262_CMD_SET_DIO_IRQ_PARAMS       0x08
#define SX1262_CMD_GET_IRQ_STATUS           0x12
#define SX1262_CMD_CLR_IRQ_STATUS           0x02
#define SX1262_CMD_SET_DIO2_AS_RF_SWITCH    0x9D
#define SX1262_CMD_SET_DIO3_AS_TCXO_CTRL    0x97

// RF Modulation and Packet commands
#define SX1262_CMD_SET_RF_FREQUENCY         0x86
#define SX1262_CMD_SET_PACKET_TYPE          0x8A
#define SX1262_CMD_GET_PACKET_TYPE          0x11
#define SX1262_CMD_SET_TX_PARAMS            0x8E
#define SX1262_CMD_SET_MODULATION_PARAMS    0x8B
#define SX1262_CMD_SET_PACKET_PARAMS        0x8C
#define SX1262_CMD_SET_CAD_PARAMS           0x88
#define SX1262_CMD_SET_BUFFER_BASE_ADDRESS  0x8F
#define SX1262_CMD_SET_LORA_SYMB_NUM_TIMEOUT 0xA0

// IRQ Flags
#define SX1262_IRQ_TX_DONE                  0x0001
#define SX1262_IRQ_RX_DONE                  0x0002
#define SX1262_IRQ_PREAMBLE_DETECTED        0x0004
#define SX1262_IRQ_SYNC_WORD_VALID          0x0008
#define SX1262_IRQ_HEADER_VALID             0x0010
#define SX1262_IRQ_HEADER_ERROR             0x0020
#define SX1262_IRQ_CRC_ERROR                0x0040
#define SX1262_IRQ_CAD_DONE                 0x0080
#define SX1262_IRQ_CAD_DETECTED             0x0100
#define SX1262_IRQ_TIMEOUT                  0x0200

// Packet Types
#define SX1262_PACKET_TYPE_GFSK             0x00
#define SX1262_PACKET_TYPE_LORA             0x01

// Register Addresses
#define SX1262_REG_LORA_SYNC_WORD_MSB       0x0740
#define SX1262_REG_LORA_SYNC_WORD_LSB       0x0741
#define SX1262_REG_RANDOM_NUMBER_GEN        0x0819
#define SX1262_REG_TX_MODULATION            0x0889
#define SX1262_REG_RX_GAIN                  0x08AC
#define SX1262_REG_TX_CLAMP_CFG             0x08D8
#define SX1262_REG_OCP_CONFIGURATION        0x08E7
#define SX1262_REG_XTA_TRIM                 0x0911
#define SX1262_REG_XTB_TRIM                 0x0912

// Bandwidths
typedef enum {
    LORA_BW_7_8 = 0x00,
    LORA_BW_10_4 = 0x08,
    LORA_BW_15_6 = 0x01,
    LORA_BW_20_8 = 0x09,
    LORA_BW_31_25 = 0x02,
    LORA_BW_41_7 = 0x0A,
    LORA_BW_62_5 = 0x03,
    LORA_BW_125 = 0x04,
    LORA_BW_250 = 0x05,
    LORA_BW_500 = 0x06
} sx1262_bandwidth_t;

// Coding Rates
typedef enum {
    LORA_CR_4_5 = 0x01,
    LORA_CR_4_6 = 0x02,
    LORA_CR_4_7 = 0x03,
    LORA_CR_4_8 = 0x04
} sx1262_coding_rate_t;

// Modem Mode
typedef enum {
    SX1262_MODEM_FSK = 0x00,
    SX1262_MODEM_LORA = 0x01
} sx1262_modem_mode_t;

// FSK Bitrate
typedef enum {
    FSK_BR_4800 = 4800,
    FSK_BR_9600 = 9600,
    FSK_BR_19200 = 19200,
    FSK_BR_38400 = 38400,
    FSK_BR_50000 = 50000,
    FSK_BR_100000 = 100000,
    FSK_BR_150000 = 150000,
    FSK_BR_300000 = 300000
} sx1262_fsk_bitrate_t;

// FSK Modulation Shaping
typedef enum {
    FSK_MOD_SHAPING_OFF = 0x00,
    FSK_MOD_SHAPING_BT_0_3 = 0x08,
    FSK_MOD_SHAPING_BT_0_5 = 0x09,
    FSK_MOD_SHAPING_BT_0_7 = 0x0A,
    FSK_MOD_SHAPING_BT_1_0 = 0x0B
} sx1262_fsk_mod_shaping_t;

// FSK RX Bandwidth
typedef enum {
    FSK_RX_BW_4800 = 0x1F,
    FSK_RX_BW_5800 = 0x17,
    FSK_RX_BW_7300 = 0x0F,
    FSK_RX_BW_9700 = 0x1E,
    FSK_RX_BW_11700 = 0x16,
    FSK_RX_BW_14600 = 0x0E,
    FSK_RX_BW_19500 = 0x1D,
    FSK_RX_BW_23400 = 0x15,
    FSK_RX_BW_29300 = 0x0D,
    FSK_RX_BW_39000 = 0x1C,
    FSK_RX_BW_46900 = 0x14,
    FSK_RX_BW_58600 = 0x0C,
    FSK_RX_BW_78200 = 0x1B,
    FSK_RX_BW_93800 = 0x13,
    FSK_RX_BW_117300 = 0x0B,
    FSK_RX_BW_156200 = 0x1A,
    FSK_RX_BW_187200 = 0x12,
    FSK_RX_BW_234300 = 0x0A,
    FSK_RX_BW_312000 = 0x19,
    FSK_RX_BW_373600 = 0x11,
    FSK_RX_BW_467000 = 0x09
} sx1262_fsk_rx_bw_t;

// Configuration structure
typedef struct {
    sx1262_modem_mode_t modem_mode; // LoRa or FSK
    uint32_t frequency;             // Frequency in Hz
    int8_t tx_power;                // TX Power in dBm
    
    // LoRa specific parameters
    sx1262_bandwidth_t bandwidth;     // Bandwidth (LoRa only)
    uint8_t spreading_factor;       // Spreading Factor 5-12 (LoRa only)
    sx1262_coding_rate_t coding_rate;// Coding Rate (LoRa only)
    bool iq_inverted;               // IQ inverted (LoRa only)
    bool rx_gain_boosted;           // true = Boosted RX Gain (+3dB sensitivity)
    
    // FSK specific parameters
    uint32_t fsk_bitrate;           // Bitrate in bps (FSK only)
    uint32_t fsk_fdev;              // Frequency deviation in Hz (FSK only)
    sx1262_fsk_rx_bw_t fsk_rx_bw;   // RX Bandwidth (FSK only)
    sx1262_fsk_mod_shaping_t fsk_shaping; // Pulse shaping (FSK only)
    
    // Common parameters
    uint16_t preamble_length;       // Preamble length
    uint8_t payload_length;         // Payload length (0 = variable)
    bool crc_on;                    // CRC enabled
    uint16_t sync_word;             // LoRa Sync Word (0x1424 = public, 0x3444 = private, 0 = default)
} sx1262_config_t;

typedef struct {
    int16_t rssi_pkt;      // RSSI of the packet in dBm
    float snr_pkt;         // SNR in dB
    int16_t signal_rssi;   // Signal RSSI in dBm
} sx1262_packet_status_t;

// Function declarations

// ============================================================================
// PHASE 1: HARDWARE INITIALIZATION (once at startup)
// ============================================================================
// Initializes: GPIO, SPI, hardware reset, basic chip configuration
// MUST be called before all other functions!
esp_err_t sx1262_init(void);

// ============================================================================
// PHASE 2: LORA CONFIGURATION (callable any time)
// ============================================================================
// Sets all LoRa parameters at once (efficient, atomic)
// Can be called any time to change parameters
esp_err_t sx1262_configure(const sx1262_config_t *config);

// ============================================================================
// PHASE 3: COMMUNICATION
// ============================================================================
esp_err_t sx1262_send(uint8_t *data, uint8_t len);
esp_err_t sx1262_receive(uint8_t *data, uint8_t *len, uint32_t timeout_ms);

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
esp_err_t sx1262_sleep(void);
esp_err_t sx1262_standby(void);
int16_t sx1262_get_rssi(void);
esp_err_t sx1262_get_packet_status(sx1262_packet_status_t *status);
esp_err_t sx1262_get_chip_info(void);

#endif // SX1262_H