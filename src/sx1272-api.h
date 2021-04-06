/**
 * \addtogroup sx1272
 */

/**
 * @{
 *
 * \file
 *         SX1272 Commands helper declaration file
 * \author
 *         Perale Thomas <tperale@vub.be>
 */
#ifndef SX1272_API_H_
#define SX1272_API_H_

#include <stdlib.h>
#include "lora.h"
#include "spi.h"
#include "sx1272-reg.h"
#include "sys/_stdint.h"

#define SX127X_RF_MID_BAND_THRESH        (525000000UL)          /**< Mid-band threshold */

#if SX127X_VERSION == SX1272_VERSION
#define SX127X_RSSI_OFFSET                                 (-139)
#else
#define SX127X_RSSI_OFFSET_LF                              (-164)
#define SX127X_RSSI_OFFSET_HF                              (-157)
#endif

typedef enum {
  sx1272_mode_sleep          = 0x00,
  sx1272_mode_standby        = 0x01,
  sx1272_mode_synthesizer_tx = 0x02,
  sx1272_mode_transmitter    = 0x03,
  sx1272_mode_synthesizer_rx = 0x04,
  sx1272_mode_receiver       = 0x05,
  sx1272_mode_receiver_single= 0x06,
  sx1272_mode_cad            = 0x07,
} sx1272_mode;

typedef enum {
  sx1272_rx_off,
  sx1272_rx_listening,
  sx1272_rx_receiving,
  sx1272_rx_received,
} sx1272_rx_mode;

typedef struct {
    spi_device_t* spi;
    uint16_t rx_rssi;
    uint16_t rx_snr;
    uint16_t rx_length;
    rtimer_clock_t rx_timestamp;
    rtimer_clock_t receiv_timestamp;
    lora_radio_t lora;
    sx1272_mode mode;
    sx1272_rx_mode rx;
    uint8_t packet[256];
} sx1272_t;

void sx127x_sleep(sx1272_t* dev);

spi_status_t sx1272_write_register(const spi_device_t *dev, uint8_t reg, uint8_t data);
spi_status_t sx1272_write_fifo(const spi_device_t *dev, uint8_t* data, uint16_t len);
spi_status_t sx1272_read_fifo(const spi_device_t *dev, uint8_t* out, uint16_t len);
uint8_t sx1272_read_register(const spi_device_t *dev, uint8_t reg);

void sx1272_set_payload_max_length(sx1272_t* dev);
void sx127x_set_opmode(sx1272_t* dev, sx1272_mode mode);
/**
 * @desc
 *
 * @param channel Channel frequency in Hz
 */
void sx127x_disable_interrupts(sx1272_t *dev);
uint32_t sx127x_get_channel(sx1272_t* dev);
int16_t sx127x_get_rssi(sx1272_t* dev);
int8_t sx127x_get_snr(sx1272_t* dev);
void sx127x_set_channel(sx1272_t* dev, uint32_t freq);
void sx127x_set_modem(sx1272_t* dev, radio_modulating_mode modem);
void sx127x_set_tx_power(sx1272_t* dev, radio_pwr pwr);
void sx127x_set_bandwidth(sx1272_t* dev, radio_bw bw);
void sx127x_set_spreading_factor(sx1272_t* dev, radio_sf sf);
void sx127x_set_coding_rate(sx1272_t* dev, radio_cr cr);
void sx127x_set_crc(sx1272_t* dev, bool crc);
void sx127x_set_iq_invert(sx1272_t* dev, bool iq);
void sx127x_set_preamble_length(sx1272_t* dev, uint16_t length);
void sx127x_set_fixed_header_len_mode(sx1272_t* dev, bool fixed);
void sx127x_set_payload_length(sx1272_t* dev, uint8_t length);
uint8_t sx127x_get_version(sx1272_t* dev);
void sx127x_init(sx1272_t* dev);
#endif /* RN2483_API_H_ */
/** @} */
