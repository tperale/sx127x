#include "contiki.h"
#include "lora.h"
#include "radio.h"
#include "rtimer-arch.h"
#include "rtimer.h"
#include "spi.h"
#include "sx1272-api.h"
#include "sx1272.h"
#include "sx1272-reg.h"
#include "sys/_stdint.h"
#include "sys/log.h"
#include "os/net/packetbuf.h"
#include "ioc.h"
#include "dev/cc2538-rf.h"
#include "net/mac/tsch/tsch.h"

#define LOG_MODULE "SX1272"
#ifndef LOG_CONF_LEVEL_SX1272
#define LOG_CONF_LEVEL_SX1272 LOG_LEVEL_INFO
#endif
#define LOG_LEVEL LOG_CONF_LEVEL_SX1272

#define BUFFER_SIZE 256

spi_device_t sx1272_spi = {
  .spi_controller = SX1272_SPI_CONTROLLER,
  .pin_spi_sck = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX1272_SPI_SCK_PORT, SX1272_SPI_SCK),
  .pin_spi_miso = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX1272_SPI_MISO_PORT, SX1272_SPI_MISO),
  .pin_spi_mosi = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX1272_SPI_MOSI_PORT, SX1272_SPI_MOSI),
  .pin_spi_cs = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX1272_SPI_CS_PORT, SX1272_SPI_CS),
  .spi_bit_rate = SX1272_SPI_BITRATE,
  .spi_pha = SX1272_SPI_PHASE,
  .spi_pol = SX1272_SPI_POL,
};

sx1272_t __sx1272_dev = {
  .spi = &sx1272_spi,
  .rx_rssi = 0,
  .rx_snr = 0,
  .rx_length = 0,
  .lora = {
    .mod = RADIO_MODULATING_LORA,
    .freq = 868100000,
    .sf = RADIO_SF_7,
    .bw = RADIO_BW_125,
    .cr = RADIO_CR_4_5,
    .prlen = 8,
    .pwr = RADIO_PWR_14,
    .implicit_header = false,
    .crc = true,
    .iqi = false,
    .wdt = 0,
    .rx_continuous = false,
    .chan = 0,
  }
};

 int tsch_packet_duration(size_t len)
{
  return US_TO_RTIMERTICKS(t_packet(&(SX1272_DEV.lora), len));
} 

/* TSCH timeslot timing (microseconds) */
tsch_timeslot_timing_usec tsch_timing_sx1272 = {
  SX1272_TSCH_DEFAULT_TS_CCA_OFFSET, /* tsch_ts_cca_offset */
  SX1272_TSCH_DEFAULT_TS_CCA, /* tsch_ts_cca */
  SX1272_TSCH_DEFAULT_TS_TX_OFFSET, /* tsch_ts_tx_offset */
  SX1272_TSCH_DEFAULT_TS_RX_OFFSET, /* tsch_ts_rx_offset */
  SX1272_TSCH_DEFAULT_TS_RX_ACK_DELAY, /* tsch_ts_rx_ack_delay */
  SX1272_TSCH_DEFAULT_TS_TX_ACK_DELAY, /* tsch_ts_tx_ack_delay */
  SX1272_TSCH_DEFAULT_TS_RX_WAIT, /* tsch_ts_rx_wait */
  SX1272_TSCH_DEFAULT_TS_ACK_WAIT, /* tsch_ts_ack_wait */
  SX1272_TSCH_DEFAULT_TS_RX_TX, /* tsch_ts_rx_tx */
  SX1272_TSCH_DEFAULT_TS_MAX_ACK, /* tsch_ts_max_ack */
  SX1272_TSCH_DEFAULT_TS_MAX_TX, /* tsch_ts_max_tx */
  SX1272_TSCH_DEFAULT_TS_TIMESLOT_LENGTH, /* tsch_ts_timeslot_length */
};

static int sx1272_receiving_packet(void);

static void sx1272_rx_internal_set(sx1272_t* dev, sx1272_rx_mode rx) {
  switch (rx) {
    case sx1272_rx_receiving:
      if (dev->rx != sx1272_rx_listening) {
        LOG_ERR("[rx_state] Went from '%d' directly to to 'receiving'\n", dev->rx);
      }
      dev->rx = sx1272_rx_receiving;
      break;
    case sx1272_rx_received:
      if (dev->rx != sx1272_rx_receiving) {
        LOG_WARN("[rx_state] Went to 'received' without 'receiving'\n");
      }
      dev->rx = sx1272_rx_received;
      break;
    default:
      dev->rx = rx;
  }
}

static int
sx1272_prepare(const void *payload, unsigned short payload_len) {
  LOG_DBG("Prepare %d bytes\n", payload_len);
  sx127x_set_payload_length(&SX1272_DEV, payload_len);;
  sx1272_write_register(SX1272_DEV.spi, REG_LR_FIFOTXBASEADDR, 0);
  sx1272_write_register(SX1272_DEV.spi, REG_LR_FIFOADDRPTR, 0);
  if (SX1272_DEV.mode == sx1272_mode_sleep) {
    sx127x_set_opmode(&SX1272_DEV, sx1272_mode_standby);
  }
  sx1272_write_fifo(SX1272_DEV.spi, (uint8_t*) payload, payload_len);
  return RADIO_RESULT_OK;
}

static int
sx1272_transmit(unsigned short payload_len) {
  sx127x_set_opmode(&SX1272_DEV, sx1272_mode_transmitter);
  while(!(sx1272_read_register(SX1272_DEV.spi, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_TXDONE));
  sx1272_write_register(SX1272_DEV.spi, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
  sx127x_set_opmode(&SX1272_DEV, sx1272_mode_standby);
  LOG_DBG("Transmit %d bytes\n", payload_len);
  return RADIO_TX_OK;
}

static int
sx1272_send(const void *payload, unsigned short payload_len) {
  sx1272_prepare(payload, payload_len);
  sx1272_transmit(payload_len);
  return RADIO_TX_OK;
}

static int
sx1272_pending_packet(void) {
  if (SX1272_DEV.rx == sx1272_rx_received) {
    return true;
  } else if (SX1272_DEV.rx == sx1272_rx_listening) {
    sx1272_receiving_packet();
    return false;
  }

  uint8_t flags;
  flags = sx1272_read_register(SX1272_DEV.spi, REG_LR_IRQFLAGS);
  if (flags & RFLR_IRQFLAGS_RXDONE){
    SX1272_DEV.rx_timestamp = RTIMER_NOW();
    sx1272_write_register(SX1272_DEV.spi, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_VALIDHEADER);
    sx1272_rx_internal_set(&SX1272_DEV, sx1272_rx_received);
    LOG_DBG("Received packet\n");
  }


  return SX1272_DEV.rx == sx1272_rx_received;
}

static int
sx1272_receiving_packet(void) {
  if (SX1272_DEV.rx == sx1272_rx_receiving) {
    if (sx1272_pending_packet()) {
      return false;
    }
    return true;
  }

  sx127x_set_opmode(&SX1272_DEV, sx1272_mode_cad);

  while ((sx1272_read_register(SX1272_DEV.spi, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDONE) != RFLR_IRQFLAGS_CADDONE);

  if (sx1272_read_register(SX1272_DEV.spi, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDETECTED) {
    SX1272_DEV.receiv_timestamp = RTIMER_NOW(); // - US_TO_RTIMERTICKS(439)
    sx1272_write_register(SX1272_DEV.spi, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);
    sx1272_rx_internal_set(&SX1272_DEV, sx1272_rx_receiving);
    sx127x_set_opmode(&SX1272_DEV, sx1272_mode_receiver);
  } else {
    sx1272_write_register(SX1272_DEV.spi, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);
  }

  return SX1272_DEV.rx == sx1272_rx_receiving;
}

static int
sx1272_read_packet(void *buf, unsigned short bufsize) {
  if (!sx1272_pending_packet()) {
    return 0;
  }

  SX1272_DEV.rx_rssi = sx127x_get_rssi(&SX1272_DEV);

  SX1272_DEV.rx_length = sx1272_read_register(SX1272_DEV.spi, REG_LR_RXNBBYTES);
  uint8_t last_rx_addr = sx1272_read_register(SX1272_DEV.spi, REG_LR_FIFORXCURRENTADDR);
  sx1272_write_register(SX1272_DEV.spi, REG_LR_FIFOADDRPTR, last_rx_addr);
  sx1272_read_fifo(SX1272_DEV.spi, buf, SX1272_DEV.rx_length < bufsize ? SX1272_DEV.rx_length : bufsize );
  if (SX1272_DEV.rx_length < bufsize) {
    ((uint8_t*) buf)[SX1272_DEV.rx_length] = '\0';
  }
  LOG_INFO("Received packet of %d bytes\n", SX1272_DEV.rx_length);

  if (SX1272_DEV.lora.rx_continuous) {
    sx1272_rx_internal_set(&SX1272_DEV, sx1272_rx_listening);
    sx127x_set_opmode(&SX1272_DEV, sx1272_mode_receiver);
  } else {
    sx1272_rx_internal_set(&SX1272_DEV, sx1272_rx_off);
    sx127x_set_opmode(&SX1272_DEV, sx1272_mode_standby);
  }

  return SX1272_DEV.rx_length;
}

static int
sx1272_on(void) {
  sx1272_rx_internal_set(&SX1272_DEV, sx1272_rx_listening);
  return 1;
}

static int
sx1272_off(void) {
  sx127x_set_opmode(&SX1272_DEV, sx1272_mode_standby);
  return 1;
}

radio_result_t sx1272_get_value(radio_param_t param, radio_value_t *value){
  if(!value) {
    return RADIO_RESULT_INVALID_VALUE;
  }

  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    *value = SX1272_DEV.mode == sx1272_mode_standby ? RADIO_POWER_MODE_OFF : RADIO_POWER_MODE_ON;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CHANNEL:
    switch (SX1272_DEV.lora.freq) {
    case 868100000:
      *value = 0;
      break;
    case 868300000:
      *value = 1;
      break;
    case 868500000:
      *value = 2;
      break;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    if(SX1272_DEV.lora.rx_continuous) {
      *value |= RADIO_RX_MODE_POLL_MODE;
    }
    if(SX1272_DEV.lora.rx_auto_ack) {
      *value |= RADIO_RX_MODE_AUTOACK;
    }
    if(SX1272_DEV.lora.rx_address_filter) {
      *value |= RADIO_RX_MODE_ADDRESS_FILTER;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    if(SX1272_DEV.lora.tx_cca) {
      *value |= RADIO_TX_MODE_SEND_ON_CCA;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    *value = SX1272_DEV.lora.pwr;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    /*
     * Clear channel assessment threshold in dBm. This threshold
     * determines the minimum RSSI level at which the radio will assume
     * that there is a packet in the air.
     */
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_RSSI:
    /* Return the RSSI value in dBm */
    return RADIO_RESULT_NOT_SUPPORTED;
  case RADIO_PARAM_LAST_RSSI:
    /* RSSI of the last packet received */
    *value = SX1272_DEV.rx_rssi;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_LAST_LINK_QUALITY:
    /* LQI of the last packet received */
    *value = SX1272_DEV.rx_snr;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MIN:
    *value = RADIO_CHANNEL_0;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MAX:
    *value = RADIO_CHANNEL_2;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MIN:
    *value = RADIO_PWR_MINUS_3;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MAX:
    *value = RADIO_PWR_15;
    return RADIO_RESULT_OK;
  case RADIO_CONST_MAX_PAYLOAD_LEN:
    *value = (radio_value_t) LORAMAC_MAX_PAYLOAD;
    return RADIO_RESULT_OK;
  case RADIO_CONST_PHY_OVERHEAD:
    *value = SX1272_DEV.lora.prlen + (SX1272_DEV.lora.crc ? 5 : 0);
    return RADIO_RESULT_OK;
  case RADIO_CONST_BYTE_AIR_TIME:
    *value = 0;
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_TX:
    *value = 0;
    switch(SX1272_SPI_BITRATE) {
      case 8000000:
        *value = US_TO_RTIMERTICKS(60 // internal time documented in datasheet
            + 122 // time  to switch from standby mode to transmit mode
            + 80
        );
        break;
      default:
        LOG_ERR("Bitrate not supported\n");
    }
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_RX:
    *value = 0;
    switch(SX1272_SPI_BITRATE) {
      case 8000000:
        *value = US_TO_RTIMERTICKS(
            71
            + 250 // Time to set opmode RX
        );
        break;
      default:
        LOG_ERR("Bitrate not supported\n");
    }
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_DETECT:
    *value = US_TO_RTIMERTICKS(2 * t_sym(SX1272_DEV.lora.sf, SX1272_DEV.lora.bw));
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }

  return RADIO_RESULT_OK;
}

  /** Set a radio parameter value. */
radio_result_t sx1272_set_value(radio_param_t param, radio_value_t value){
  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    if(value == RADIO_POWER_MODE_ON) {
      sx1272_on();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_OFF) {
      sx1272_off();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_CARRIER_ON ||
       value == RADIO_POWER_MODE_CARRIER_OFF) {
      return RADIO_RESULT_NOT_SUPPORTED;
    }
    return RADIO_RESULT_INVALID_VALUE;
  case RADIO_PARAM_CHANNEL:
    sx127x_set_opmode(&SX1272_DEV, sx1272_mode_sleep);
    switch (value) {
    case RADIO_CHANNEL_0:
      sx127x_set_channel(&SX1272_DEV, 868100000);
      break;
      case RADIO_CHANNEL_1:
      sx127x_set_channel(&SX1272_DEV, 868300000);
      break;
    case RADIO_CHANNEL_2:
      sx127x_set_channel(&SX1272_DEV, 868500000);
      break;
    }
    sx127x_set_opmode(&SX1272_DEV, sx1272_mode_standby);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    if(value & ~(RADIO_RX_MODE_ADDRESS_FILTER | RADIO_RX_MODE_AUTOACK | RADIO_RX_MODE_POLL_MODE)) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    /* RN2483_DEV.radio.rx_continuous = (value & RADIO_RX_MODE_POLL_MODE) != 0; */
    /* RN2483_DEV.radio.rx_auto_ack = (value & RADIO_RX_MODE_AUTOACK) != 0; */
    /* RN2483_DEV.radio.rx_address_filter = (value & RADIO_RX_MODE_ADDRESS_FILTER) != 0; */
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    /* RN2483_DEV.radio.tx_cca = (value & RADIO_TX_MODE_SEND_ON_CCA) != 0; */
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    if(value < RADIO_PWR_MINUS_3 || value > RADIO_PWR_15) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    /* Find the closest higher PA_LEVEL for the desired output power */
    sx127x_set_tx_power(&SX1272_DEV, value);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    /*
     * Clear channel assessment threshold in dBm. This threshold
     * determines the minimum RSSI level at which the radio will assume
     * that there is a packet in the air.
     *
     * The CCA threshold must be set to a level above the noise floor of
     * the deployment. Otherwise mechanisms such as send-on-CCA and
     * low-power-listening duty cycling protocols may not work
     * correctly. Hence, the default value of the system may not be
     * optimal for any given deployment.
     */
    return RADIO_RESULT_NOT_SUPPORTED;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
  return RADIO_RESULT_OK;
}

radio_result_t sx1272_get_object(radio_param_t param, void *dest, size_t size){

  switch(param) {
  case RADIO_PARAM_LAST_PACKET_TIMESTAMP:
    if(size != sizeof(rtimer_clock_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    LOG_DBG("LORA COMM -> %d us of length %d bytes\n", t_packet(&(SX1272_DEV.lora), SX1272_DEV.rx_length), SX1272_DEV.rx_length);
    *(rtimer_clock_t *)dest = SX1272_DEV.rx_timestamp - US_TO_RTIMERTICKS(
      t_packet(&(SX1272_DEV.lora), SX1272_DEV.rx_length)
      + 622 // Delay between TX end of transmission and RX detection of end of transmission
      + 152 // Delay between interrupt on DIO1 and software detection
    );
    return RADIO_RESULT_OK;
  case RADIO_CONST_TSCH_TIMING:
    if(size != sizeof(uint32_t *) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *((uint32_t **)dest) = tsch_timing_sx1272;
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }

  return RADIO_RESULT_OK;
}

/**
 * Set a radio parameter object. The memory area referred to by the
 * argument 'src' will not be accessed after the function returns.
 */
radio_result_t sx1272_set_object(radio_param_t param, const void *src, size_t size) {
  return RADIO_RESULT_OK;
}

#define CCA_CLEAR 1
#define CCA_BUSY 0
int sx1272_clear_channel_assesment(){
  return CCA_CLEAR;
}

int sx1272_reset() {
  GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SX1272_RESET_GPIO_PORT), GPIO_PIN_MASK(SX1272_RESET_GPIO));
  GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SX1272_RESET_GPIO_PORT), GPIO_PIN_MASK(SX1272_RESET_GPIO));
  GPIO_CLR_PIN(PIN_TO_PORT_BASE(SX1272_RESET_GPIO_PORT), GPIO_PIN_MASK((SX1272_RESET_GPIO)));
  clock_delay_usec(1000);
  ioc_set_over(SX1272_RESET_GPIO_PORT, SX1272_RESET_GPIO, IOC_OVERRIDE_OE);
  GPIO_SET_PIN(GPIO_PORT_TO_BASE(SX1272_RESET_GPIO_PORT), GPIO_PIN_MASK(SX1272_RESET_GPIO));
  clock_delay_usec(110);
  ioc_set_over(SX1272_RESET_GPIO_PORT, SX1272_RESET_GPIO, IOC_OVERRIDE_OE);
  GPIO_CLR_PIN(GPIO_PORT_TO_BASE(SX1272_RESET_GPIO_PORT), GPIO_PIN_MASK(SX1272_RESET_GPIO));
  ioc_set_over(SX1272_RESET_GPIO_PORT, SX1272_RESET_GPIO, IOC_OVERRIDE_PUE);
  clock_delay_usec(5500);

  return 0;
}

int sx1272_init() {
  LOG_DBG("Init SPI\n");
  spi_acquire(SX1272_DEV.spi);

  if (sx127x_get_version(&SX1272_DEV) != SX127X_VERSION) {
    LOG_ERR("Error communicating with the module through SPI (version not valid)\n");
    spi_release(SX1272_DEV.spi);
    return RADIO_RESULT_ERROR;
  }

  LOG_DBG("Reset Module\n");
  sx1272_reset();

  sx127x_set_opmode(&SX1272_DEV, sx1272_mode_sleep);

  sx127x_init(&SX1272_DEV);

  return RADIO_RESULT_OK;
}

const struct radio_driver sx1272_radio_driver = {
  sx1272_init,
  sx1272_prepare,
  sx1272_transmit,
  sx1272_send,
  sx1272_read_packet,
  sx1272_clear_channel_assesment,
  sx1272_receiving_packet,
  sx1272_pending_packet,
  sx1272_on,
  sx1272_off,
  sx1272_get_value,
  sx1272_set_value,
  sx1272_get_object,
  sx1272_set_object,
};
