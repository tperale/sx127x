#ifndef LORA_INCLUDE_H_
#define LORA_INCLUDE_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief   Default device EUI
 *
 *          8 bytes key, required for join procedure
 */
#ifndef LORAMAC_DEV_EUI_DEFAULT
#define LORAMAC_DEV_EUI_DEFAULT        { 0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00 }
#endif

/**
 * @brief   Default application EUI
 *
 *          8 bytes key, required for join procedure
 */
#ifndef LORAMAC_APP_EUI_DEFAULT
#define LORAMAC_APP_EUI_DEFAULT        { 0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00 }
#endif

/**
 * @brief   Default application key
 *
 *          16 bytes key, required for join procedure
 */
#ifndef LORAMAC_APP_KEY_DEFAULT
#define LORAMAC_APP_KEY_DEFAULT        { 0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00 }
#endif

/**
 * @brief   Default application session key
 *
 *          16 bytes key, only required for ABP join procedure type
 */
#ifndef LORAMAC_APP_SKEY_DEFAULT
#define LORAMAC_APP_SKEY_DEFAULT       { 0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00 }
#endif

/**
 * @brief   Default network session key
 *
 *          16 bytes key, only required for ABP join procedure type.
 */
#ifndef LORAMAC_NWK_SKEY_DEFAULT
#define LORAMAC_NWK_SKEY_DEFAULT       { 0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00, \
                                         0x00, 0x00, 0x00, 0x00 }
#endif

/**
 * @brief   Default device address
 */
#ifndef LORAMAC_DEV_ADDR_DEFAULT
#define LORAMAC_DEV_ADDR_DEFAULT       { 0x00, 0x00, 0x00, 0x00 }
#endif
/** @} */

/**
 * @name    LoRaMAC default values
 * @{
 */
/**
 * @brief   Default device class (A, B or C)
 */
#ifndef LORAMAC_DEFAULT_DEVICE_CLASS
#define LORAMAC_DEFAULT_DEVICE_CLASS           (LORAMAC_CLASS_A)
#endif

/**
 * @brief   Default NetID (only valid with ABP join procedure)
 */
#ifndef LORAMAC_DEFAULT_NETID
#define LORAMAC_DEFAULT_NETID                  (1U)
#endif

/**
 * @brief   Default network type (public or private)
 */
#ifndef LORAMAC_DEFAULT_PUBLIC_NETWORK
#define LORAMAC_DEFAULT_PUBLIC_NETWORK         (true)
#endif
/**
 * @brief   Default datarate (only valid for EU)
 */
#ifndef LORAMAC_DEFAULT_DR
#define LORAMAC_DEFAULT_DR                     (LORAMAC_DR_0)
#endif

/**
 * @brief   Default MAC TX power (14dBm in EU)
 */
#ifndef LORAMAC_DEFAULT_TX_POWER
#define LORAMAC_DEFAULT_TX_POWER               (LORAMAC_TX_PWR_1)
#endif

/**
 * @brief   Default MAC TX port (from 1 to 223)
 */
#ifndef LORAMAC_DEFAULT_TX_PORT
#define LORAMAC_DEFAULT_TX_PORT                (2U)
#endif

/**
 * @brief   Default MAC TX mode (confirmable or unconfirmable)
 */
#ifndef LORAMAC_DEFAULT_TX_MODE
#define LORAMAC_DEFAULT_TX_MODE               (LORAMAC_TX_CNF)
#endif

/**
 * @brief   Default MAC TX power (14dBm in EU)
 */
#ifndef LORAMAC_DEFAULT_TX_POWER
#define LORAMAC_DEFAULT_TX_POWER               (LORAMAC_TX_PWR_1)
#endif

/**
 * @brief   Default adaptive datarate state
 */
#ifndef LORAMAC_DEFAULT_ADR
#define LORAMAC_DEFAULT_ADR                    (false)
#endif

/**
 * @brief   Default uplink retransmission
 */
#ifndef LORAMAC_DEFAULT_RETX
#define LORAMAC_DEFAULT_RETX                   (5U)
#endif

/**
 * @brief   Default link check interval (in seconds)
 *
 *          0 means the link check process is disabled
 */
#ifndef LORAMAC_DEFAULT_LINKCHK
#define LORAMAC_DEFAULT_LINKCHK                (0U)
#endif

/**
 * @brief   Default first RX window delay (in ms)
 */
#ifndef LORAMAC_DEFAULT_RX1_DELAY
#define LORAMAC_DEFAULT_RX1_DELAY              (1000U)
#endif

/**
 * @brief   Default second RX window delay (in ms)
 */
#define LORAMAC_DEFAULT_RX2_DELAY              (1000U + LORAMAC_DEFAULT_RX1_DELAY)

/**
 * @brief   Default automatic reply status
 */
#ifndef LORAMAC_DEFAULT_AR
#define LORAMAC_DEFAULT_AR                     (false)
#endif

/**
 * @brief   Default second RX window datarate index
 */
#ifndef LORAMAC_DEFAULT_RX2_DR
#define LORAMAC_DEFAULT_RX2_DR                 (LORAMAC_DR_0)
#endif

/**
 * @brief   Default second RX window frequency (in Hz)
 */
#ifndef LORAMAC_DEFAULT_RX2_FREQ
#define LORAMAC_DEFAULT_RX2_FREQ               (869525000UL)
#endif

/**
 * @brief   Default LoRaMAC join procedure
 */
#ifndef LORAMAC_DEFAULT_JOIN_PROCEDURE
#define LORAMAC_DEFAULT_JOIN_PROCEDURE         (LORAMAC_MODE_OTAA)
#endif

/**
 * @brief   Default LoRaMAC join accept delay 1 (in seconds)
 */
#ifndef LORAMAC_DEFAULT_JOIN_DELAY1
#define LORAMAC_DEFAULT_JOIN_DELAY1            (5U)
#endif

/**
 * @brief   Default LoRaMAC join accept delay 2
 */
#ifndef LORAMAC_DEFAULT_JOIN_DELAY2
#define LORAMAC_DEFAULT_JOIN_DELAY2            (6U)
#endif

/**
 * @brief   Default max FCNT gap
 */
#ifndef LORAMAC_DEFAULT_MAX_FCNT_GAP
#define LORAMAC_DEFAULT_MAX_FCNT_GAP           (16384U)
#endif

/**
 * @brief   Default adaptive datarate ACK limit (in s)
 */
#ifndef LORAMAC_DEFAULT_ADR_ACK_LIMIT
#define LORAMAC_DEFAULT_ADR_ACK_LIMIT          (64U)
#endif

/**
 * @brief   Default adaptive datarate ACK delay (in s)
 */
#ifndef LORAMAC_DEFAULT_ADR_ACK_DELAY
#define LORAMAC_DEFAULT_ADR_ACK_DELAY          (32U)
#endif

/**
 * @brief   Default adaptive datarate timeout
 */
#ifndef LORAMAC_DEFAULT_ADR_TIMEOUT
#define LORAMAC_DEFAULT_ADR_TIMEOUT            (3U)
#endif

/**
 * @brief   Default maximum system overall timing error
 */
#ifndef LORAMAC_DEFAULT_SYSTEM_MAX_RX_ERROR
#define LORAMAC_DEFAULT_SYSTEM_MAX_RX_ERROR    (50)
#endif

/**
 * @brief   Default minimum RX symbols to detect a frame
 */
#ifndef LORAMAC_DEFAULT_MIN_RX_SYMBOLS
#define LORAMAC_DEFAULT_MIN_RX_SYMBOLS         (12)
#endif

/**
 * @brief   Default minimum RX symbols to detect a frame
 */
#ifndef LORAMAC_MAX_PAYLOAD
#define LORAMAC_MAX_PAYLOAD                    (243)
#endif
/** @} */

/**
 * @name    LoRaMAC constants
 * @{
 */
/**
 * @brief   Device EUI length in bytes
 */
#define LORAMAC_DEVEUI_LEN             (8U)

/**
 * @brief   Device address length in bytes
 */
#define LORAMAC_DEVADDR_LEN            (4U)

/**
 * @brief   Application EUI length in bytes
 */
#define LORAMAC_APPEUI_LEN             (8U)

/**
 * @brief   Application key length in bytes
 */
#define LORAMAC_APPKEY_LEN             (16U)

/**
 * @brief   Application session key length in bytes
 */
#define LORAMAC_APPSKEY_LEN            (16U)

/**
 * @brief   Network session key length in bytes
 */
#define LORAMAC_NWKSKEY_LEN            (16U)

/**
 * @brief   Minimum port value
 */
#define LORAMAC_PORT_MIN               (1U)

/**
 * @brief   Maximmu port value
 */
#define LORAMAC_PORT_MAX               (223U)

/**
 * @brief Application Nonce length in bytes
 */
#define LORAMAC_APP_NONCE_LEN          (3U)

/**
 * @brief Network ID length in bytes
 */
#define LORAMAC_NETWORK_ID_LEN         (3U)

/** @} */

/**
 * @name    LoRaMAC parameters indexes
 */

/**
 * @brief   Device frequency band
 */
typedef enum {
  LORAMAC_BAND_433 = 433,
  LORAMAC_BAND_868 = 868,
} loramac_band;

/**
 * @brief   Payload type
 */
typedef enum {
  LORAMAC_PAYLOAD_TYPE_CNF,
  LORAMAC_PAYLOAD_TYPE_UNCNF,
} loramac_payload_type;

/**
 * @brief   Device join procedure type
 */
typedef enum {
  LORAMAC_MODE_OTAA,                     /**< Over The Air ACtivation */
  LORAMAC_MODE_ABP,                      /**< Activation by personalization */
} loramac_mode;

/**
 * @brief   Device class
 */
typedef enum {
  LORAMAC_CLASS_A,                     /**< Class A device */
  LORAMAC_CLASS_B,                     /**< Class B device */
  LORAMAC_CLASS_C,                     /**< Class C device */
} loramac_class_t;

/**
 * @brief   LoRaMAC transmission mode
 */
typedef enum {
  LORAMAC_TX_CNF,                      /**< Confirmable transmission mode */
  LORAMAC_TX_UNCNF,                    /**< Unconfirmable transmission mode */
} loramac_tx_mode_t;

/**
 * @brief   LoRaMAC datarate indexes
 *
 *          Each index corresponds to a different modulation, spreading factor
 *          and bandwidth depending on the regions.
 */
typedef enum {
  /**
   * - ISM EU863-870: LoRa modulation, SF12, BW125 (250bit/s)
   * - ISM US902-928: LoRa modulation, SF10, BW125 (980bit/s)
   * - ISM CN779-787: LoRa modulation, SF12, BW125 (250bit/s)
   * - ISM EU433: LoRa modulation, SF12, BW125 (250bit/s)
   * - ISM AU915-928: LoRa modulation, SF10, BW125 (980bit/s)
   * - ISM CN470-510: LoRa modulation, SF12, BW125 (250bit/s)
   * - ISM AS923: LoRa modulation, SF12, BW125 (250bit/s)
   * - ISM KR920-923: LoRa modulation, SF12, BW125 (250bit/s)
   *
   * Default value used.
   */
  LORAMAC_DR_0 = 0,
  /**
   * - ISM EU863-870: LoRa modulation, SF11, BW125 (440bit/s)
   * - ISM US902-928: LoRa modulation, SF9, BW125 (1760bit/s)
   * - ISM CN779-787: LoRa modulation, SF11, BW125 (440bit/s)
   * - ISM EU433: LoRa modulation, SF11, BW125 (440bit/s)
   * - ISM AU915-928: LoRa modulation, SF9, BW125 (1760bit/s)
   * - ISM CN470-510: LoRa modulation, SF11, BW125 (440bit/s)
   * - ISM AS923: LoRa modulation, SF11, BW125 (440bit/s)
   * - ISM KR920-923: LoRa modulation, SF11, BW125 (440bit/s)
   */
  LORAMAC_DR_1,
  /**
   * - ISM EU863-870: LoRa modulation, SF10, BW125 (980bit/s)
   * - ISM US902-928: LoRa modulation, SF8, BW125 (3125bit/s)
   * - ISM CN779-787: LoRa modulation, SF10, BW125 (980bit/s)
   * - ISM EU433: LoRa modulation, SF10, BW125 (980bit/s)
   * - ISM AU915-928: LoRa modulation, SF8, BW125 (3125bit/s)
   * - ISM CN470-510: LoRa modulation, SF10, BW125 (980bit/s)
   * - ISM AS923: LoRa modulation, SF10, BW125 (980bit/s)
   * - ISM KR920-923: LoRa modulation, SF10, BW125 (980bit/s)
   */
  LORAMAC_DR_2,
  /**
   * - ISM EU863-870: LoRa modulation, SF9, BW125 (1760bit/s)
   * - ISM US902-928: LoRa modulation, SF7, BW125 (5470bit/s)
   * - ISM CN779-787: LoRa modulation, SF9, BW125 (1760bit/s)
   * - ISM EU433: LoRa modulation, SF9, BW125 (1760bit/s)
   * - ISM AU915-928: LoRa modulation, SF7, BW125 (5470bit/s)
   * - ISM CN470-510: LoRa modulation, SF9, BW125 (1760bit/s)
   * - ISM AS923: LoRa modulation, SF9, BW125 (1760bit/s)
   * - ISM KR920-923: LoRa modulation, SF9, BW125 (1760bit/s)
   */
  LORAMAC_DR_3,
  /**
   * - ISM EU863-870: LoRa modulation, SF8, BW125 (3125bit/s)
   * - ISM US902-928: LoRa modulation, SF8, BW500 (12500bit/s)
   * - ISM CN779-787: LoRa modulation, SF8, BW125 (3125bit/s)
   * - ISM EU433: LoRa modulation, SF8, BW125 (3125bit/s)
   * - ISM AU915-928: LoRa modulation, SF8, BW500 (12500bit/s)
   * - ISM CN470-510: LoRa modulation, SF8, BW125 (3125bit/s)
   * - ISM AS923: LoRa modulation, SF8, BW125 (3125bit/s)
   * - ISM KR920-923: LoRa modulation, SF8, BW125 (3125bit/s)
   */
  LORAMAC_DR_4,
  /**
   * - ISM EU863-870: LoRa modulation, SF7, BW125 (5470bit/s)
   * - ISM US902-928: reserved for future use
   * - ISM CN779-787: LoRa modulation, SF7, BW125 (5470bit/s)
   * - ISM EU433: LoRa modulation, SF7, BW125 (5470bit/s)
   * - ISM AU915-928: reserved for future use
   * - ISM CN470-510: LoRa modulation, SF7, BW125 (5470bit/s)
   * - ISM AS923: LoRa modulation, SF7, BW125 (5470bit/s)
   * - ISM KR920-923: LoRa modulation, SF7, BW125 (5470bit/s)
   */
  LORAMAC_DR_5,
  /**
   * - ISM EU863-870: LoRa modulation, SF7, BW250 (11000bit/s)
   * - ISM US902-928: reserved for future use
   * - ISM CN779-787: LoRa modulation, SF7, BW250 (11000bit/s)
   * - ISM EU433 : LoRa modulation, SF7, BW250 (11000bit/s)
   * - ISM AU915-928: reserved for future use
   * - ISM CN470-510: reserved for future use
   * - ISM AS923: LoRa modulation, SF7, BW250 (11000bit/s)
   * - ISM KR920-923: reserved for future use
   */
  LORAMAC_DR_6,
  /**
   * - ISM EU863-870: FSK modulation (50000bit/s)
   * - ISM US902-928: reserved for future use
   * - ISM CN779-787: FSK modulation (50000bit/s)
   * - ISM EU433: FSK modulation (50000bit/s)
   * - ISM AU915-928: reserved for future use
   * - ISM CN470-510: reserved for future use
   * - ISM AS923: FSK modulation (50000bit/s)
   * - ISM KR920-923: reserved for future use
   */
  LORAMAC_DR_7,
  /**
   * - ISM EU863-870: reserved for future use
   * - ISM US902-928: LoRa modulation, SF12, BW500 (980bit/s)
   * - ISM CN779-787: reserved for future use
   * - ISM EU433: reserved for future use
   * - ISM AU915-928: LoRa modulation, SF12, BW500 (980bit/s)
   * - ISM CN470-510: reserved for future use
   * - ISM AS923: reserved for future use
   * - ISM KR920-923: reserved for future use
   */
  LORAMAC_DR_8,
  /**
   * - ISM EU863-870: reserved for future use
   * - ISM US902-928: LoRa modulation, SF11, BW500 (1760bit/s)
   * - ISM CN779-787: reserved for future use
   * - ISM EU433: reserved for future use
   * - ISM AU915-928: LoRa modulation, SF11, BW500 (1760bit/s)
   * - ISM CN470-510: reserved for future use
   * - ISM AS923: reserved for future use
   * - ISM KR920-923: reserved for future use
   */
  LORAMAC_DR_9,
  /**
   * - ISM EU863-870: reserved for future use
   * - ISM US902-928: LoRa modulation, SF10, BW500 (3900bit/s)
   * - ISM CN779-787: reserved for future use
   * - ISM EU433: reserved for future use
   * - ISM AU915-928: LoRa modulation, SF10, BW500 (3900bit/s)
   * - ISM CN470-510: reserved for future use
   * - ISM AS923: reserved for future use
   * - ISM KR920-923: reserved for future use
   */
  LORAMAC_DR_10,
  /**
   * - ISM EU863-870: reserved for future use
   * - ISM US902-928: LoRa modulation, SF9, BW500 (7000bit/s)
   * - ISM CN779-787: reserved for future use
   * - ISM EU433: reserved for future use
   * - ISM AU915-928: LoRa modulation, SF9, BW500 (7000bit/s)
   * - ISM CN470-510: reserved for future use
   * - ISM AS923: reserved for future use
   * - ISM KR920-923: reserved for future use
   */
  LORAMAC_DR_11,
  /**
   * - ISM EU863-870: reserved for future use
   * - ISM US902-928: LoRa modulation, SF8, BW500 (12500bit/s)
   * - ISM CN779-787: reserved for future use
   * - ISM EU433: reserved for future use
   * - ISM AU915-928: LoRa modulation, SF8, BW500 (12500bit/s)
   * - ISM CN470-510: reserved for future use
   * - ISM AS923: reserved for future use
   * - ISM KR920-923: reserved for future use
   */
  LORAMAC_DR_12,
  /**
   * - ISM EU863-870: reserved for future use
   * - ISM US902-928: LoRa modulation, SF7, BW500 (21900bit/s)
   * - ISM CN779-787: reserved for future use
   * - ISM EU433: reserved for future use
   * - ISM AU915-928: LoRa modulation, SF7, BW500 (21900bit/s)
   * - ISM CN470-510: reserved for future use
   * - ISM AS923: reserved for future use
   * - ISM KR920-923: reserved for future use
   */
  LORAMAC_DR_13,
  /**
   * - ISM EU863-870: reserved for future use
   * - ISM US902-928: reserved for future use
   * - ISM CN779-787: reserved for future use
   * - ISM EU433: reserved for future use
   * - ISM AU915-928: reserved for future use
   * - ISM CN470-510: reserved for future use
   * - ISM AS923: reserved for future use
   * - ISM KR920-923: reserved for future use
   */
  LORAMAC_DR_14,
  /**
   * - ISM EU863-870: reserved for future use
   * - ISM US902-928: reserved for future use
   * - ISM CN779-787: reserved for future use
   * - ISM EU433: reserved for future use
   * - ISM AU915-928: reserved for future use
   * - ISM CN470-510: reserved for future use
   * - ISM AS923: reserved for future use
   * - ISM KR920-923: reserved for future use
   */
  LORAMAC_DR_15,
} loramac_dr_idx_t;

/**
 * @brief   LoRaMAC transmission power indexes
 *
 *          Each index corresponds to a different modulation, spreading factor
 *          and bandwidth depending on the regions.
 */
typedef enum {
  /**
   * - EU863-870: 20dBm (if supported)
   * - US902-928: 30dBm (if supported)
   * - CN779-787: 10dBm
   * - EU433: 10dBm
   * - AU915-928: 30dBm
   * - CN470-510: 17dBm
   * - ISM AS923: 14dBm
   * - ISM KR920-923: 20dBm
   */
  LORAMAC_TX_PWR_0 = 0,
  /**
   * - EU863-870: 14dBm
   * - US902-928: 28dBm
   * - CN779-787: 7dBm
   * - EU433: 7dBm
   * - AU915-928: 28dBm
   * - CN470-510: 16dBm
   * - ISM AS923: 12dBm
   * - ISM KR920-923: 14dBm
   *
   * Default value used
   */
  LORAMAC_TX_PWR_1,
  /**
   * - EU863-870: 11dBm
   * - US902-928: 26dBm
   * - CN779-787: 4dBm
   * - EU433: 4dBm
   * - AU915-928: 26dBm
   * - CN470-510: 14dBm
   * - ISM AS923: 10dBm
   * - ISM KR920-923: 10dBm
   */
  LORAMAC_TX_PWR_2,
  /**
   * - EU863-870: 8dBm
   * - US902-928: 24dBm
   * - CN779-787: 1dBm
   * - EU433: 1dBm
   * - AU915-928: 24dBm
   * - CN470-510: 12dBm
   * - ISM AS923: 8dBm
   * - ISM KR920-923: 8dBm
   */
  LORAMAC_TX_PWR_3,
  /**
   * - EU863-870: 5dBm
   * - US902-928: 22dBm
   * - CN779-787: -2dBm
   * - EU433: -2dBm
   * - AU915-928: 22dBm
   * - CN470-510: 10dBm
   * - ISM AS923: 6dBm
   * - ISM KR920-923: 5dBm
   */
  LORAMAC_TX_PWR_4,
  /**
   * - EU863-870: 2dBm
   * - US902-928: 20dBm
   * - CN779-787: -5dBm
   * - EU433: -5dBm
   * - AU915-928: 20dBm
   * - CN470-510: 7dBm
   * - ISM AS923: 4dBm
   * - ISM KR920-923: 2dBm
   */
  LORAMAC_TX_PWR_5,
  /**
   * - EU863-870: Reserved for future use
   * - US902-928: 18dBm
   * - CN779-787: Reserved for future use
   * - EU433: Reserved for future use
   * - AU915-928: 18dBm
   * - CN470-510: 5dBm
   * - ISM AS923: Reserved for future use
   * - ISM KR920-923: 0dBm
   */
  LORAMAC_TX_PWR_6,
  /**
   * - EU863-870: Reserved for future use
   * - US902-928: 16dBm
   * - CN779-787: Reserved for future use
   * - EU433: Reserved for future use
   * - AU915-928: 16dBm
   * - CN470-510: 2dBm
   * - ISM AS923: Reserved for future use
   * - ISM KR920-923: Reserved for future use
   */
  LORAMAC_TX_PWR_7,
  /**
   * - EU863-870: Reserved for future use
   * - US902-928: 14dBm
   * - CN779-787: Reserved for future use
   * - EU433: Reserved for future use
   * - AU915-928: 14dBm
   * - CN470-510: Reserved for future use
   * - ISM AS923: Reserved for future use
   * - ISM KR920-923: Reserved for future use
   */
  LORAMAC_TX_PWR_8,
  /**
   * - EU863-870: Reserved for future use
   * - US902-928: 12dBm
   * - CN779-787: Reserved for future use
   * - EU433: Reserved for future use
   * - AU915-928: 12dBm
   * - CN470-510: Reserved for future use
   * - ISM AS923: Reserved for future use
   * - ISM KR920-923: Reserved for future use
   */
  LORAMAC_TX_PWR_9,
  /**
   * - EU863-870: Reserved for future use
   * - US902-928: 10dBm
   * - CN779-787: Reserved for future use
   * - EU433: Reserved for future use
   * - AU915-928: 10dBm
   * - CN470-510: Reserved for future use
   * - ISM AS923: Reserved for future use
   * - ISM KR920-923: Reserved for future use
   */
  LORAMAC_TX_PWR_10,
  /**
   * - EU863-870: Reserved for future use
   * - US902-928: Reserved for future use
   * - CN779-787: Reserved for future use
   * - EU433: Reserved for future use
   * - AU915-928: Reserved for future use
   * - CN470-510: Reserved for future use
   * - ISM AS923: Reserved for future use
   * - ISM KR920-923: Reserved for future use
   */
  LORAMAC_TX_PWR_11,
  /**
   * - EU863-870: Reserved for future use
   * - US902-928: Reserved for future use
   * - CN779-787: Reserved for future use
   * - EU433: Reserved for future use
   * - AU915-928: Reserved for future use
   * - CN470-510: Reserved for future use
   * - ISM AS923: Reserved for future use
   * - ISM KR920-923: Reserved for future use
   */
  LORAMAC_TX_PWR_12,
  /**
   * - EU863-870: Reserved for future use
   * - US902-928: Reserved for future use
   * - CN779-787: Reserved for future use
   * - EU433: Reserved for future use
   * - AU915-928: Reserved for future use
   * - CN470-510: Reserved for future use
   * - ISM AS923: Reserved for future use
   * - ISM KR920-923: Reserved for future use
   */
  LORAMAC_TX_PWR_13,
  /**
   * - EU863-870: Reserved for future use
   * - US902-928: Reserved for future use
   * - CN779-787: Reserved for future use
   * - EU433: Reserved for future use
   * - AU915-928: Reserved for future use
   * - CN470-510: Reserved for future use
   * - ISM AS923: Reserved for future use
   * - ISM KR920-923: Reserved for future use
   */
  LORAMAC_TX_PWR_14,
  /**
   * - EU863-870: Reserved for future use
   * - US902-928: Reserved for future use
   * - CN779-787: Reserved for future use
   * - EU433: Reserved for future use
   * - AU915-928: Reserved for future use
   * - CN470-510: Reserved for future use
   * - ISM AS923: Reserved for future use
   * - ISM KR920-923: Reserved for future use
   */
  LORAMAC_TX_PWR_15,
} loramac_tx_pwr_idx_t;
/** @} */

/**
 * @brief   RN2483
 */
typedef enum {
  /**
   * - Frequency (868): 868.1 MHz
   * - Frequency (433): 433.175 MHz
   * - Duty Cycle: 302
   * - Duty Rate Rage: 0-5
   */
  RADIO_CHANNEL_0 = 0,
  /**
   * - Frequency (868): 868.3 MHz
   * - Frequency (433): 433.375 MHz
   * - Duty Cycle: 302
   * - Duty Rate Rage: 0-5
   */
  RADIO_CHANNEL_1,
  /**
   * - Frequency (868): 868.5 MHz
   * - Frequency (433): 433.575 MHz
   * - Duty Cycle: 302
   * - Duty Rate Rage: 0-5
   */
  RADIO_CHANNEL_2,
  RADIO_CHANNEL_NO_CHANNEL,
} radio_channel;

/**
 * @brief   RN2483 Radio Automatic Frequency Correction Bandwidth
 */
typedef enum {
  RADIO_AFCBW_250,
  RADIO_AFCBW_125,
  RADIO_AFCBW_62_5,
  RADIO_AFCBW_31_3,
  RADIO_AFCBW_15_6,
  RADIO_AFCBW_7_8,
  RADIO_AFCBW_3_9,
  RADIO_AFCBW_200,
  RADIO_AFCBW_100,
  RADIO_AFCBW_50,
  RADIO_AFCBW_25,
  RADIO_AFCBW_12_5,
  RADIO_AFCBW_6_3,
  RADIO_AFCBW_3_1,
  RADIO_AFCBW_166_7,
  RADIO_AFCBW_83_3,
  RADIO_AFCBW_41_7,
  RADIO_AFCBW_20_8,
  RADIO_AFCBW_10_4,
  RADIO_AFCBW_5_2,
  RADIO_AFCBW_2_6
} radio_afcbw;

/**
 * @brief   RN2483 Radio data shaping
 */
typedef enum {
  RADIO_BT_NONE,
  RADIO_BT_1_0,
  RADIO_BT_0_5,
  RADIO_BT_0_3
} radio_bt;

/**
 * @brief   RN2483 Radio BandWidth
 */
typedef enum {
  RADIO_BW_125 = 125,
  RADIO_BW_250 = 250,
  RADIO_BW_500 = 500
} radio_bw;

/**
 * @brief   RN2483 Radio Coding Rate
 */
typedef enum {
  RADIO_CR_4_5 = 1,
  RADIO_CR_4_6,
  RADIO_CR_4_7,
  RADIO_CR_4_8
} radio_cr;

/**
 * @brief   RN2483 Radio Modulating mode
 */
typedef enum {
  RADIO_MODULATING_FSK,
  RADIO_MODULATING_LORA
} radio_modulating_mode;

/**
 * @brief   RN2483 Radio Transmit output power.
 */
typedef enum {
  RADIO_PWR_MINUS_3 = -3,
  RADIO_PWR_MINUS_2 = -2,
  RADIO_PWR_MINUS_1 = -1,
  RADIO_PWR_0 = 0,
  RADIO_PWR_1,
  RADIO_PWR_2,
  RADIO_PWR_3,
  RADIO_PWR_4,
  RADIO_PWR_5,
  RADIO_PWR_6,
  RADIO_PWR_7,
  RADIO_PWR_8,
  RADIO_PWR_9,
  RADIO_PWR_10,
  RADIO_PWR_11,
  RADIO_PWR_12,
  RADIO_PWR_13,
  RADIO_PWR_14,
  RADIO_PWR_15
} radio_pwr;

/**
 * @brief   RN2483 Radio Reception Signal Bandwidth
 */
typedef enum {
  RADIO_RXBW_250,
  RADIO_RXBW_125,
  RADIO_RXBW_62_5,
  RADIO_RXBW_31_3,
  RADIO_RXBW_15_6,
  RADIO_RXBW_7_8,
  RADIO_RXBW_3_9,
  RADIO_RXBW_200,
  RADIO_RXBW_100,
  RADIO_RXBW_50,
  RADIO_RXBW_25,
  RADIO_RXBW_12_5,
  RADIO_RXBW_6_3,
  RADIO_RXBW_3_1,
  RADIO_RXBW_166_7,
  RADIO_RXBW_83_3,
  RADIO_RXBW_41_7,
  RADIO_RXBW_20_8,
  RADIO_RXBW_10_4,
  RADIO_RXBW_5_2,
  RADIO_RXBW_2_6
} radio_rxbw;

/**
 * @brief   RN2483 Radio Spreading Factor
 */
typedef enum {
  RADIO_SF_7 = 7,
  RADIO_SF_8,
  RADIO_SF_9,
  RADIO_SF_10,
  RADIO_SF_11,
  RADIO_SF_12
} radio_sf;



typedef struct {
  radio_modulating_mode mod;
  radio_channel chan;
  uint32_t freq;
  radio_sf sf;
  radio_bw bw;
  radio_cr cr;
  uint32_t prlen;
  radio_pwr pwr;
  bool crc;
  bool iqi;
  bool implicit_header;
  uint32_t wdt;
  bool rx_continuous;
  bool rx_auto_ack;
  bool rx_address_filter;
  bool tx_cca;
} lora_radio_t;

int t_sym(lora_radio_t *radio);
int t_preamble(lora_radio_t *radio);
int payload_sym_nb(lora_radio_t *radio, size_t len) ;
int t_payload(lora_radio_t *radio, size_t len);
int t_packet(lora_radio_t *radio, size_t len);

#endif /* LORA_INCLUDE_H_ */
