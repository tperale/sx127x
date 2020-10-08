/**
 * \file
 *         SX1272 Commands function helper implementation
 * \author
 *         Perale Thomas <tperale@vub.be>
 */

/**
 * \addtogroup sx1272
 * @{
*/
#include "sx1272-api.h"
#include "os/dev/radio.h"
#include "spi.h"
#include "sx1272-reg.h"
#include "sys/_stdint.h"
#include "sys/log.h"

#include <string.h>
#include <stdlib.h>
#include <assert.h>

#define LOG_MODULE "SX1272-API"
#ifndef LOG_CONF_LEVEL_SX1272_API
#define LOG_CONF_LEVEL_SX1272_API LOG_LEVEL_INFO
#endif
#define LOG_LEVEL LOG_CONF_LEVEL_SX1272_API

#define FREQ_STEP (61.03515625)

spi_status_t sx1272_write_register(const spi_device_t *dev, uint8_t reg, uint8_t data) {
  spi_select(dev);
  uint8_t ret;
  if ((ret = spi_write_byte(dev, reg | 0x80)) != SPI_DEV_STATUS_OK) {
    LOG_ERR("Error writting address %02x\n", ret);
    return ret;
  }

  if ((ret = spi_write_byte(dev, data)) != SPI_DEV_STATUS_OK) {
    LOG_ERR("Error writting data %02x\n", ret);
    return ret;
  }
  spi_deselect(dev);

  return SPI_DEV_STATUS_OK;
}

spi_status_t sx1272_write_fifo(const spi_device_t *dev, uint8_t* data, uint16_t len) {
  spi_select(dev);
  spi_write_byte(dev, REG_LR_FIFO | 0x80);
  spi_write(dev, data, len);
  spi_deselect(dev);

  return SPI_DEV_STATUS_OK;
}

spi_status_t sx1272_read_fifo(const spi_device_t *dev, uint8_t* out, uint16_t len) {
  spi_select(dev);
  spi_write_byte(dev, REG_LR_FIFO);
  spi_read(dev, out, len);
  spi_deselect(dev);

  return SPI_DEV_STATUS_OK;
}



uint8_t sx1272_read_register(const spi_device_t *dev, uint8_t reg) {
  spi_select(dev);
  uint8_t ret;
  spi_read_register(dev, reg, &ret, 1);
  spi_deselect(dev);

  return ret;
}



/* static spi_status_t sx1272_read_register(const spi_device_t *dev, uint8_t reg, uint8_t* data) { */
/*   uint8_t ret; */ 
/*   if ((ret = spi_select(dev)) != SPI_DEV_STATUS_OK) { */
/*     return ret; */
/*   } */

/*   if ((ret = spi_read_register(dev, reg & 0x7F, data, 1)) != SPI_DEV_STATUS_OK) { */
/*     return ret; */
/*   } */

/*   if ((ret = spi_deselect(dev)) != SPI_DEV_STATUS_OK) { */
/*     return ret; */
/*   } */

/*   return SPI_DEV_STATUS_OK; */
/* } */

void sx127x_disable_interrupts(sx1272_t* dev) {
  sx1272_write_register(dev->spi, REG_LR_IRQFLAGSMASK, 
    RFLR_IRQFLAGS_RXTIMEOUT 
      | RFLR_IRQFLAGS_RXDONE
      | RFLR_IRQFLAGS_PAYLOADCRCERROR 
      | RFLR_IRQFLAGS_VALIDHEADER
      | RFLR_IRQFLAGS_TXDONE 
      | RFLR_IRQFLAGS_CADDONE
      | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL 
      | RFLR_IRQFLAGS_CADDETECTED
  );
}

uint32_t sx127x_get_channel(sx1272_t* dev) {
  uint8_t msb;
  uint8_t mid;
  uint8_t lsb;

  msb = sx1272_read_register(dev->spi, REG_LR_FRFMSB);
  mid = sx1272_read_register(dev->spi, REG_LR_FRFMID);
  lsb = sx1272_read_register(dev->spi, REG_LR_FRFLSB);
  uint32_t temp = ((uint32_t)(msb << 16) | (mid << 8) | lsb);
  return (uint32_t)(((double) temp) * ((double) FREQ_STEP));
}

void sx127x_set_channel(sx1272_t* dev, uint32_t freq) {
  dev->lora.freq = freq;
  freq = (uint32_t)((double) freq / (double) FREQ_STEP);
  sx1272_write_register(dev->spi, REG_LR_FRFMSB, (uint8_t)((freq >> 16) & 0xFF));
  sx1272_write_register(dev->spi, REG_LR_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
  sx1272_write_register(dev->spi, REG_LR_FRFLSB, (uint8_t)(freq & 0xFF));
}

void sx127x_set_opmode(sx1272_t* dev, sx1272_mode mode) {
  uint8_t opmode; 
  uint8_t previous = dev->mode;
  dev->mode = mode;

  switch (dev->mode) {
    case sx1272_mode_sleep:
      if (previous != mode) {
        LOG_DBG("Set op mode: SLEEP\n");
      }
      break;
    case sx1272_mode_standby:
      if (previous != mode) {
        LOG_DBG("Set op mode: STANDBY\n");
      }
      break;
    case sx1272_mode_synthesizer_tx:
      if (previous != mode) {
        LOG_DBG("Set op mode: SYNTHESIZER TX\n");
      }
      break;
    case sx1272_mode_synthesizer_rx:
      if (previous != mode) {
        LOG_DBG("Set op mode: SYNTHESIZER RX\n");
      }
      break;
    case sx1272_mode_transmitter:
      if (previous != mode) {
        LOG_DBG("Set op mode: TX\n");
      }
      /* Enable TXDONE interrupt */
      sx1272_write_register(dev->spi, REG_LR_IRQFLAGSMASK,
        RFLR_IRQFLAGS_RXTIMEOUT |
        RFLR_IRQFLAGS_RXDONE |
        RFLR_IRQFLAGS_PAYLOADCRCERROR |
        RFLR_IRQFLAGS_VALIDHEADER |
        /* RF_LORA_IRQFLAGS_TXDONE | */
        RFLR_IRQFLAGS_CADDONE |
        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
        RFLR_IRQFLAGS_CADDETECTED);

      /* Set TXDONE interrupt to the DIO0 line */
      sx1272_write_register(dev->spi, REG_LR_DIOMAPPING1,
        RFLR_DIOMAPPING1_DIO0_01
      );
      break;
    case sx1272_mode_receiver:
    case sx1272_mode_receiver_single:
      if (dev->lora.rx_continuous) {
        if (previous != mode) {
          LOG_DBG("Set op mode: RX\n");
        }
      } else {
        if (previous != mode) {
          LOG_DBG("Set op mode: RX Single\n");
        }
      }
      sx1272_write_register(dev->spi, REG_LR_IRQFLAGSMASK, 
        /* RFLR_IRQFLAGS_RXTIMEOUT */ 
          /* | RFLR_IRQFLAGS_RXDONE */
          /* | RFLR_IRQFLAGS_PAYLOADCRCERROR */ 
          /* | RFLR_IRQFLAGS_VALIDHEADER */
          RFLR_IRQFLAGS_TXDONE 
          | RFLR_IRQFLAGS_CADDONE
          | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL 
          | RFLR_IRQFLAGS_CADDETECTED
      );
      /* Set RXDONE interrupt to the DIO0 line */
      sx1272_write_register(dev->spi, REG_LR_DIOMAPPING1, 
        RFLR_DIOMAPPING1_DIO0_00
        | RFLR_DIOMAPPING1_DIO1_00
        | RFLR_DIOMAPPING1_DIO3_01
      );
      sx1272_write_register(dev->spi, REG_LR_FIFORXBASEADDR, 0);
      sx1272_write_register(dev->spi, REG_LR_FIFOADDRPTR, 0);
      break;
    case sx1272_mode_cad:
      if (previous != mode) {
        LOG_DBG("Set op mode: CAD\n");
      }
      break;
    default:
      if (previous != mode) {
        LOG_ERR("Set op mode: Unknown\n");
      }
      return;
  }

  opmode = sx1272_read_register(dev->spi, REG_LR_OPMODE);
  sx1272_write_register(dev->spi, REG_LR_OPMODE, (opmode & RFLR_OPMODE_MASK) | mode);
}

void sx127x_set_modem(sx1272_t* dev, radio_modulating_mode modem) {
  switch (modem) {
    default:
    case RADIO_MODULATING_LORA: {
      uint8_t opmode; 
      opmode = sx1272_read_register(dev->spi, REG_LR_OPMODE);
      LOG_DBG("OPMODE: %02x\n", opmode);
      sx1272_write_register(dev->spi, REG_LR_OPMODE, (opmode & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON); // MOSI: 0x81 --> MISO: 0x8F
      sx1272_write_register(dev->spi, REG_LR_DIOMAPPING1, 0x00);
      sx1272_write_register(dev->spi, REG_LR_DIOMAPPING2, 0x00);
      break;
    }
    case RADIO_MODULATING_FSK:
      // Unimplemented
      LOG_ERR("FSK Modulation not supported\n");
      break;
  }
}

// TODO improve because the use of PABOOST is not clear
void sx127x_set_tx_power(sx1272_t* dev, radio_pwr pwr) {
  int8_t power = pwr;
  uint8_t pa_config;
  uint8_t pa_dac;

  pa_config = sx1272_read_register(dev->spi, REG_LR_PACONFIG);
  pa_dac = sx1272_read_register(dev->spi, REG_LR_PADAC);

  pa_config = (pa_config & RFLR_PACONFIG_PASELECT_MASK) | RFLR_PACONFIG_PASELECT_RFO;

  if ((pa_config & RFLR_PACONFIG_PASELECT_PABOOST) == RFLR_PACONFIG_PASELECT_PABOOST) {
    if (power > 17) {
      pa_dac = (pa_dac & RFLR_PADAC_20DBM_MASK) | RFLR_PADAC_20DBM_ON;
    } else {
      pa_dac = (pa_dac & RFLR_PADAC_20DBM_MASK) | RFLR_PADAC_20DBM_OFF;
    }

    if ((pa_dac & RFLR_PADAC_20DBM_ON) == RFLR_PADAC_20DBM_ON) {
      if (power < 5) {
          power = 5;
      }
      if (power > 20) {
          power = 20;
      }
      pa_config = (pa_config & RFLR_PACONFIG_OUTPUTPOWER_MASK)
                  | (uint8_t)((uint16_t)(power - 5) & 0x0F);
    } else {
      if (power < 2) {
          power = 2;
      }
      if (power > 17) {
          power = 17;
      }
      pa_config = (pa_config & RFLR_PACONFIG_OUTPUTPOWER_MASK)
                  | (uint8_t)((uint16_t)(power - 2) & 0x0F);
    }
  } else {
    if (power < -1) {
        power = -1;
    }
    if (power > 14) {
        power = 14;
    }
    pa_config = (pa_config & RFLR_PACONFIG_OUTPUTPOWER_MASK)
                | (uint8_t)((uint16_t)(power + 1) & 0x0F);
  }

  sx1272_write_register(dev->spi, REG_LR_PACONFIG, pa_config);
  sx1272_write_register(dev->spi, REG_LR_PADAC, pa_dac);
}

void sx127x_set_bandwidth(sx1272_t* dev, radio_bw bw) {
  uint8_t modem1;

  dev->lora.bw = bw;

  modem1 = sx1272_read_register(dev->spi, REG_LR_MODEMCONFIG1);
  modem1 &= RFLR_MODEMCONFIG1_BW_MASK;

  switch (bw) {
    case RADIO_BW_125:
      modem1 |= RFLR_MODEMCONFIG1_BW_125_KHZ;
      break;
    case RADIO_BW_250:
      modem1 |= RFLR_MODEMCONFIG1_BW_250_KHZ;
      break;
    case RADIO_BW_500:
      modem1 |= RFLR_MODEMCONFIG1_BW_500_KHZ;
      break;
  }
  sx1272_write_register(dev->spi, REG_LR_MODEMCONFIG1, modem1);
}

void sx127x_set_spreading_factor(sx1272_t* dev, radio_sf sf) {
  if (sf < 7 || sf > 12) {
    LOG_ERR("SF%d is not supported by this driver\n", sf);
  }

  uint8_t modem2;
  modem2 = sx1272_read_register(dev->spi, REG_LR_MODEMCONFIG2);
  modem2 &= RFLR_MODEMCONFIG2_SF_MASK;
  modem2 |= (sf << 4);
  sx1272_write_register(dev->spi, REG_LR_MODEMCONFIG2, modem2);
}

void sx127x_set_coding_rate(sx1272_t* dev, radio_cr cr) {
  uint8_t modem1;
  modem1 = sx1272_read_register(dev->spi, REG_LR_MODEMCONFIG1);
  modem1 &= RFLR_MODEMCONFIG1_CODINGRATE_MASK;
#if SX127X_VERSION == SX1272_VERSION
  modem1 |= (cr << 3);
#else
  modem1 |= (cr << 1);
#endif
  sx1272_write_register(dev->spi, REG_LR_MODEMCONFIG1, modem1);
}

void sx127x_set_crc(sx1272_t* dev, bool crc) {
#if SX127X_VERSION == SX1272_VERSION
  uint8_t modem1;
  modem1 = sx1272_read_register(dev->spi, REG_LR_MODEMCONFIG1);
  modem1 &= RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK;
  modem1 |= (crc << 1);
  sx1272_write_register(dev->spi, REG_LR_MODEMCONFIG1, modem1);
#else
  uint8_t modem2;
  modem2 = sx1272_read_register(dev->spi, REG_LR_MODEMCONFIG2);
  modem2 &= RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK;
  modem2 |= (crc << 2);
  sx1272_write_register(dev->spi, REG_LR_MODEMCONFIG2, modem2);
#endif
}

int16_t sx127x_get_rssi(sx1272_t* dev) {
  int16_t rssi = 0;
#if SX127X_VERSION == SX1272_VERSION
  rssi = SX127X_RSSI_OFFSET + sx1272_read_register(dev->spi, REG_LR_RSSIVALUE);
#else /* MODULE_SX1276 */
  if (dev->lora.freq > SX127X_RF_MID_BAND_THRESH) {
    rssi = SX127X_RSSI_OFFSET_HF + sx1272_read_register(dev->spi, REG_LR_RSSIVALUE);
  }
  else {
    rssi = SX127X_RSSI_OFFSET_LF + sx1272_read_register(dev->spi, REG_LR_RSSIVALUE);
  }
#endif
  return rssi;
}


int8_t sx127x_get_snr(sx1272_t* dev) {
  return sx1272_read_register(dev->spi, REG_LR_PKTSNRVALUE);
}

/* void sx127x_set_freq_hop(sx1272_t* dev, LORA_FREQUENCY_HOPPING_DEFAULT) { */
/* } */
/* void sx127x_set_hop_period(sx1272_t* dev, LORA_FREQUENCY_HOPPING_PERIOD_DEFAULT) { */
/* } */

void sx127x_set_iq_invert(sx1272_t* dev, bool iq) {
  uint8_t prev;
  prev = sx1272_read_register(dev->spi, REG_LR_INVERTIQ);
  sx1272_write_register(dev->spi, REG_LR_INVERTIQ,
      (prev & RFLR_INVERTIQ_RX_MASK & RFLR_INVERTIQ_TX_MASK)
      | RFLR_INVERTIQ_RX_OFF
      | (iq ? RFLR_INVERTIQ_TX_ON : RFLR_INVERTIQ_TX_OFF)
  );
  sx1272_write_register(dev->spi, REG_LR_INVERTIQ2,
      (iq ? RFLR_INVERTIQ2_ON : RFLR_INVERTIQ2_OFF));
}

void sx127x_set_payload_length(sx1272_t* dev, uint8_t length) {
  sx1272_write_register(dev->spi, REG_LR_PAYLOADLENGTH, length);
}

void sx127x_set_preamble_length(sx1272_t* dev, uint16_t length) {
  dev->lora.prlen = length;
  sx1272_write_register(dev->spi, REG_LR_PREAMBLEMSB, (length >> 8) & 0xFF);
  sx1272_write_register(dev->spi, REG_LR_PREAMBLELSB, length & 0xFF);
}

void sx127x_set_fixed_header_len_mode(sx1272_t* dev, bool fixed) {
  dev->lora.implicit_header = fixed;

  uint8_t modem1;
  modem1 = sx1272_read_register(dev->spi, REG_LR_MODEMCONFIG1);
  modem1 &= RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK;
#if SX127X_VERSION == SX1272_VERSION
  modem1 |= (fixed << 2);
#else
  modem1 |= fixed;
#endif
  sx1272_write_register(dev->spi, REG_LR_MODEMCONFIG1, modem1);
}

void sx127x_set_symbol_timeout(sx1272_t* dev, uint8_t timeout) {

}

uint8_t sx127x_get_version(sx1272_t* dev) {
  uint8_t version;
  /* spi_read_register(dev->spi, REG_LR_VERSION, &version, 1); */
  version = sx1272_read_register(dev->spi, REG_LR_VERSION);
  LOG_DBG("VERSION: 0x%02X\n", version);
  return version;
}

void sx127x_init(sx1272_t* dev) {
  sx127x_set_modem(dev, dev->lora.mod);
  sx127x_set_channel(dev, dev->lora.freq);
  /* sx127x_set_tx_power(dev, dev->lora.pwr); */
  sx127x_set_bandwidth(dev, dev->lora.bw);
  sx127x_set_spreading_factor(dev, dev->lora.sf);
  sx127x_set_coding_rate(dev, dev->lora.cr);
  sx127x_set_crc(dev, dev->lora.crc);
  /* sx127x_set_freq_hop(dev, ); */
  /* sx127x_set_hop_period(dev, ); */
  sx127x_set_fixed_header_len_mode(dev, dev->lora.implicit_header);
  sx127x_set_iq_invert(dev, dev->lora.iqi);
  sx127x_set_preamble_length(dev, dev->lora.prlen);
  /* sx127x_set_symbol_timeout(dev, ); */
}
/** @} */
