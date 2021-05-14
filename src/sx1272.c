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
#include "dev/leds.h"
#include "dev/gpio-hal.h"
#include "net/mac/tsch/tsch.h"
#include "watchdog.h"
#include <stdint.h>

#define LOG_MODULE "SX127X"
#ifndef LOG_CONF_LEVEL_SX127X
#define LOG_CONF_LEVEL_SX127X LOG_LEVEL_INFO
#endif
#define LOG_LEVEL LOG_CONF_LEVEL_SX127X

#define BUFFER_SIZE 256

spi_device_t sx127x_spi = {
  .spi_controller = SX127X_SPI_CONTROLLER,
  .pin_spi_sck = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX127X_SPI_SCK_PORT, SX127X_SPI_SCK),
  .pin_spi_miso = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX127X_SPI_MISO_PORT, SX127X_SPI_MISO),
  .pin_spi_mosi = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX127X_SPI_MOSI_PORT, SX127X_SPI_MOSI),
  .pin_spi_cs = GPIO_PORT_PIN_TO_GPIO_HAL_PIN(SX127X_SPI_CS_PORT, SX127X_SPI_CS),
  .spi_bit_rate = SX127X_SPI_BITRATE,
  .spi_pha = SX127X_SPI_PHASE,
  .spi_pol = SX127X_SPI_POL,
};

semtech_dev_t __sx127x_dev = {
  .spi = &sx127x_spi,
  .rx_rssi = 0,
  .rx_snr = 0,
  .rx_length = 0,
  .rx = sx127x_rx_off,
  .pending = false,
  .lora = {
    .mod = RADIO_MODULATING_LORA,
    .freq = 868100000,
    .sf = SX127X_LORA_INIT_SF,
    .bw = SX127X_LORA_INIT_BW,
    .cr = SX127X_LORA_INIT_CR,
    .prlen = SX127X_LORA_INIT_PRLEN,
    .pwr = RADIO_PWR_14,
    .implicit_header = SX127X_LORA_INIT_IMPLICIT_HEADER,
    .crc = SX127X_LORA_INIT_CRC,
    .iqi = false,
    .wdt = 0,
#if SX127X_BUSY_RX 
    .rx_continuous = false,
#else
    .rx_continuous = true,
#endif
    .chan = 0,
  }
};

int tsch_packet_duration(size_t len)
{
  return US_TO_RTIMERTICKS(t_packet(&(SX127X_DEV.lora), len));
} 

/* TSCH timeslot timing (microseconds) */
tsch_timeslot_timing_usec tsch_timing_sx127x = {
  SX127X_TSCH_DEFAULT_TS_CCA_OFFSET, /* tsch_ts_cca_offset */
  SX127X_TSCH_DEFAULT_TS_CCA, /* tsch_ts_cca */
  SX127X_TSCH_DEFAULT_TS_TX_OFFSET, /* tsch_ts_tx_offset */
  SX127X_TSCH_DEFAULT_TS_RX_OFFSET, /* tsch_ts_rx_offset */
  SX127X_TSCH_DEFAULT_TS_RX_ACK_DELAY, /* tsch_ts_rx_ack_delay */
  SX127X_TSCH_DEFAULT_TS_TX_ACK_DELAY, /* tsch_ts_tx_ack_delay */
  SX127X_TSCH_DEFAULT_TS_RX_WAIT, /* tsch_ts_rx_wait */
  SX127X_TSCH_DEFAULT_TS_ACK_WAIT, /* tsch_ts_ack_wait */
  SX127X_TSCH_DEFAULT_TS_RX_TX, /* tsch_ts_rx_tx */
  SX127X_TSCH_DEFAULT_TS_MAX_ACK, /* tsch_ts_max_ack */
  SX127X_TSCH_DEFAULT_TS_MAX_TX, /* tsch_ts_max_tx */
  SX127X_TSCH_DEFAULT_TS_TIMESLOT_LENGTH, /* tsch_ts_timeslot_length */
};

static int sx127x_receiving_packet(void);
static int sx127x_read_packet(void *buf, unsigned short bufsize);

static void sx127x_rx_internal_set(semtech_dev_t* dev, sx127x_rx_mode rx) {
  switch (rx) {
    case sx127x_rx_receiving:
      if (dev->rx != sx127x_rx_listening) {
        LOG_ERR("[rx_state] Went from '%d' directly to to 'receiving'\n", dev->rx);
      }
      dev->rx = sx127x_rx_receiving;
      break;
    case sx127x_rx_received:
#if SX127X_BUSY_RX
      if (dev->rx != sx127x_rx_receiving) {
        LOG_WARN("[rx_state] Went to 'received' without 'receiving'\n");
      }
#endif
      dev->pending = true;
      dev->rx = sx127x_rx_received;
      break;
    case sx127x_rx_read:
      if (!dev->pending) {
        LOG_WARN("[rx_state] read the content of the module without packet pending\n");
      }
      dev->pending = false;
      break;
    default:
      dev->rx = rx;
  }
}

#if SX127X_USE_INTERRUPT
#define SX127X_DIO0_PORT_BASE GPIO_PORT_TO_BASE(SX127X_DIO0_PORT)
#define SX127X_DIO0_PIN_MASK GPIO_PIN_MASK(SX127X_DIO0_PIN)
#define SX127X_DIO3_PORT_BASE GPIO_PORT_TO_BASE(SX127X_DIO3_PORT)
#define SX127X_DIO3_PIN_MASK GPIO_PIN_MASK(SX127X_DIO3_PIN)

#if SX127X_BUSY_RX
static void sx127x_interrupt_dio0(gpio_hal_pin_mask_t pin_mask) { 
}
static void sx127x_interrupt_dio3(gpio_hal_pin_mask_t pin_mask) { 
}
#else
static void sx127x_interrupt_dio0(gpio_hal_pin_mask_t pin_mask) { 
  if (SX127X_DEV.mode == sx127x_mode_receiver || SX127X_DEV.mode == sx127x_mode_receiver_single) {
    SX127X_DEV.rx_timestamp = RTIMER_NOW();
    sx127x_rx_internal_set(&SX127X_DEV, sx127x_rx_received);
    if (SX127X_DEV.lora.rx_continuous) {
      packetbuf_clear();
      int len = sx127x_read_register(SX127X_DEV.spi, REG_LR_RXNBBYTES);

      if(len > 0 && len != 3) {
        sx127x_read_packet(packetbuf_dataptr(), PACKETBUF_SIZE);
        packetbuf_set_datalen(len);
        NETSTACK_MAC.input();
      }
    }
    sx127x_disable_interrupts(&SX127X_DEV);
  }
}

static void sx127x_interrupt_dio3(gpio_hal_pin_mask_t pin_mask) { 
  if (SX127X_DEV.mode == sx127x_mode_receiver || SX127X_DEV.mode == sx127x_mode_receiver_single) {
    if (SX127X_DEV.rx != sx127x_rx_receiving) {
      SX127X_DEV.receiv_timestamp = RTIMER_NOW();
      sx127x_rx_internal_set(&SX127X_DEV, sx127x_rx_receiving);
    }
    sx127x_write_register(SX127X_DEV.spi, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_VALIDHEADER);
  } else if (SX127X_DEV.mode == sx127x_mode_cad) {
    // CAD Done
    /* sx127x_disable_interrupts(&SX127X_DEV); */
    /* sx127x_write_register(SX127X_DEV.spi, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE); */
  }
}
#endif

gpio_hal_event_handler_t sx127x_event_handler_dio0 = {
  .next = NULL,
  .handler = sx127x_interrupt_dio0,
  .pin_mask  = (gpio_hal_pin_to_mask(SX127X_DIO0_PIN) << (SX127X_DIO0_PORT << 3))
};
gpio_hal_event_handler_t sx127x_event_handler_dio3 = {
  .next = NULL,
  .handler = sx127x_interrupt_dio3,
  .pin_mask  = (gpio_hal_pin_to_mask(SX127X_DIO3_PIN) << (SX127X_DIO3_PORT << 3))
};
#endif

static int
sx127x_prepare(const void *payload, unsigned short payload_len) {
  LOG_DBG("Prepare %d bytes\n", payload_len);
  sx127x_set_payload_length(&SX127X_DEV, payload_len);;
  sx127x_write_register(SX127X_DEV.spi, REG_LR_FIFOTXBASEADDR, 0);
  sx127x_write_register(SX127X_DEV.spi, REG_LR_FIFOADDRPTR, 0);
  if (SX127X_DEV.mode == sx127x_mode_sleep || SX127X_DEV.mode == sx127x_mode_receiver) {
    sx127x_set_opmode(&SX127X_DEV, sx127x_mode_standby);
    sx127x_rx_internal_set(&__sx127x_dev, sx127x_rx_off);
  }
  sx127x_write_fifo(SX127X_DEV.spi, (uint8_t*) payload, payload_len);
  return RADIO_RESULT_OK;
}

static int
sx127x_transmit(unsigned short payload_len) {
  sx127x_set_opmode(&SX127X_DEV, sx127x_mode_transmitter);
  while(!(sx127x_read_register(SX127X_DEV.spi, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_TXDONE)) watchdog_periodic();
  sx127x_write_register(SX127X_DEV.spi, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
  sx127x_set_opmode(&SX127X_DEV, sx127x_mode_standby);
  if (SX127X_DEV.lora.rx_continuous) {
    sx127x_rx_internal_set(&SX127X_DEV, sx127x_rx_listening);
    sx127x_set_opmode(&SX127X_DEV, sx127x_mode_receiver);
  }
  LOG_DBG("Transmit %d bytes\n", payload_len);
  return RADIO_TX_OK;
}

static int
sx127x_send(const void *payload, unsigned short payload_len) {
  sx127x_prepare(payload, payload_len);
  sx127x_transmit(payload_len);
  return RADIO_TX_OK;
}

static int
sx127x_pending_packet(void) {
#if SX127X_BUSY_RX 
  if (SX127X_DEV.rx == sx127x_rx_received) {
    return true;
  } else if (SX127X_DEV.rx == sx127x_rx_listening) {
    sx127x_receiving_packet();
    return false;
  }

  uint8_t flags;
  flags = sx127x_read_register(SX127X_DEV.spi, REG_LR_IRQFLAGS);
  if (flags & RFLR_IRQFLAGS_RXDONE){
    SX127X_DEV.rx_timestamp = RTIMER_NOW() - US_TO_RTIMERTICKS(650);
    sx127x_write_register(SX127X_DEV.spi, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_VALIDHEADER);
    sx127x_rx_internal_set(&SX127X_DEV, sx127x_rx_received);
    LOG_DBG("Received packet\n");
  }
#endif

  return SX127X_DEV.pending;
}

static int
sx127x_receiving_packet(void) {
#if SX127X_BUSY_RX
  if (SX127X_DEV.rx == sx127x_rx_receiving) {
    if (sx127x_pending_packet()) {
      return false;
    }
    return true;
  }

  sx127x_set_opmode(&SX127X_DEV, sx127x_mode_cad);
  while ((sx127x_read_register(SX127X_DEV.spi, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDONE) != RFLR_IRQFLAGS_CADDONE);

  if (sx127x_read_register(SX127X_DEV.spi, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDETECTED) {
    SX127X_DEV.receiv_timestamp = RTIMER_NOW(); // - US_TO_RTIMERTICKS(439)
    sx127x_write_register(SX127X_DEV.spi, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);
    sx127x_rx_internal_set(&SX127X_DEV, sx127x_rx_receiving);
    sx127x_set_opmode(&SX127X_DEV, sx127x_mode_receiver);
  } else {
    sx127x_write_register(SX127X_DEV.spi, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);
  }
#endif

  return SX127X_DEV.rx == sx127x_rx_receiving;
}

static int
sx127x_read_packet(void *buf, unsigned short bufsize) {
  if (!sx127x_pending_packet()) {
    return 0;
  }

  SX127X_DEV.rx_rssi = sx127x_get_rssi(&SX127X_DEV);

  SX127X_DEV.rx_length = sx127x_read_register(SX127X_DEV.spi, REG_LR_RXNBBYTES);
  uint8_t last_rx_addr = sx127x_read_register(SX127X_DEV.spi, REG_LR_FIFORXCURRENTADDR);
  sx127x_write_register(SX127X_DEV.spi, REG_LR_FIFOADDRPTR, last_rx_addr);
  sx127x_read_fifo(SX127X_DEV.spi, buf, SX127X_DEV.rx_length < bufsize ? SX127X_DEV.rx_length : bufsize );
  if (SX127X_DEV.rx_length < bufsize) {
    ((uint8_t*) buf)[SX127X_DEV.rx_length] = '\0';
  }
  LOG_INFO("Received packet of %d bytes\n", SX127X_DEV.rx_length);
  sx127x_rx_internal_set(&SX127X_DEV, sx127x_rx_read);
  if (SX127X_DEV.lora.rx_continuous) {
    sx127x_rx_internal_set(&SX127X_DEV, sx127x_rx_listening);
    sx127x_set_opmode(&SX127X_DEV, sx127x_mode_receiver);
  } else {
    sx127x_rx_internal_set(&SX127X_DEV, sx127x_rx_off);
    sx127x_set_opmode(&SX127X_DEV, sx127x_mode_standby);
  }

  return SX127X_DEV.rx_length;
}

static int
sx127x_on(void) {
  sx127x_rx_internal_set(&SX127X_DEV, sx127x_rx_listening);
#if !SX127X_BUSY_RX
  // In busy reception mode the RX is triggered with a CAD 
  // scan that is faster to detect the packet than the DIO3 
  // interrupt for the valid header
  // In async reception mode the reception notification will come
  // from the interrupt.
  sx127x_set_opmode(&SX127X_DEV, sx127x_mode_receiver);
#endif
  return 1;
}

static int
sx127x_off(void) {
  sx127x_set_opmode(&SX127X_DEV, sx127x_mode_standby);
  sx127x_rx_internal_set(&SX127X_DEV, sx127x_rx_off);
  sx127x_disable_interrupts(&SX127X_DEV);
  return 1;
}

radio_result_t sx127x_get_value(radio_param_t param, radio_value_t *value){
  if(!value) {
    return RADIO_RESULT_INVALID_VALUE;
  }

  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    *value = SX127X_DEV.mode == sx127x_mode_standby ? RADIO_POWER_MODE_OFF : RADIO_POWER_MODE_ON;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CHANNEL:
    switch (SX127X_DEV.lora.freq) {
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
    if(SX127X_DEV.lora.rx_continuous) {
      *value |= RADIO_RX_MODE_POLL_MODE;
    }
    if(SX127X_DEV.lora.rx_auto_ack) {
      *value |= RADIO_RX_MODE_AUTOACK;
    }
    if(SX127X_DEV.lora.rx_address_filter) {
      *value |= RADIO_RX_MODE_ADDRESS_FILTER;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TX_MODE:
    if(SX127X_DEV.lora.tx_cca) {
      *value |= RADIO_TX_MODE_SEND_ON_CCA;
    }
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    *value = SX127X_DEV.lora.pwr;
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
    *value = SX127X_DEV.rx_rssi;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_LAST_LINK_QUALITY:
    /* LQI of the last packet received */
    *value = SX127X_DEV.rx_snr;
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
    *value = SX127X_DEV.lora.prlen + (SX127X_DEV.lora.crc ? 5 : 0);
    return RADIO_RESULT_OK;
  case RADIO_CONST_BYTE_AIR_TIME:
    *value = 0;
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_TX:
    *value = 0;
    switch(SX127X_SPI_BITRATE) {
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
    switch(SX127X_SPI_BITRATE) {
      case 8000000:
        *value = US_TO_RTIMERTICKS(
            71
            + 153 // Time to set opmode CAD
        );
        break;
      default:
        LOG_ERR("Bitrate not supported\n");
    }
    return RADIO_RESULT_OK;
  case RADIO_CONST_DELAY_BEFORE_DETECT:
    *value = US_TO_RTIMERTICKS(2 * t_sym(SX127X_DEV.lora.sf, SX127X_DEV.lora.bw));
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }

  return RADIO_RESULT_OK;
}

/** Set a radio parameter value. */
radio_result_t sx127x_set_value(radio_param_t param, radio_value_t value){
  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    if(value == RADIO_POWER_MODE_ON) {
      sx127x_on();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_OFF) {
      sx127x_off();
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_CARRIER_ON ||
       value == RADIO_POWER_MODE_CARRIER_OFF) {
      return RADIO_RESULT_NOT_SUPPORTED;
    }
    return RADIO_RESULT_INVALID_VALUE;
  case RADIO_PARAM_CHANNEL:
    sx127x_set_opmode(&SX127X_DEV, sx127x_mode_sleep);
    switch (value) {
    case RADIO_CHANNEL_0:
      sx127x_set_channel(&SX127X_DEV, 868100000);
      break;
      case RADIO_CHANNEL_1:
      sx127x_set_channel(&SX127X_DEV, 868300000);
      break;
    case RADIO_CHANNEL_2:
      sx127x_set_channel(&SX127X_DEV, 868500000);
      break;
    }
    sx127x_set_opmode(&SX127X_DEV, sx127x_mode_standby);
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    return RADIO_RESULT_OK;
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
    sx127x_set_tx_power(&SX127X_DEV, value);
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

radio_result_t sx127x_get_object(radio_param_t param, void *dest, size_t size){
  switch(param) {
  case RADIO_PARAM_LAST_PACKET_TIMESTAMP:
    if(size != sizeof(rtimer_clock_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    LOG_DBG("LORA COMM -> %d us of length %d bytes\n", t_packet(&(SX127X_DEV.lora), SX127X_DEV.rx_length), SX127X_DEV.rx_length);
    *(rtimer_clock_t *)dest = SX127X_DEV.rx_timestamp - US_TO_RTIMERTICKS(
      t_packet(&(SX127X_DEV.lora), SX127X_DEV.rx_length)
      + 622 // Delay between TX end of transmission and RX detection of end of transmission
      + 152 // Delay between interrupt on DIO1 and software detection
    );
    return RADIO_RESULT_OK;
  case RADIO_CONST_TSCH_TIMING:
    if(size != sizeof(uint32_t *) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *((uint32_t **)dest) = tsch_timing_sx127x;
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
radio_result_t sx127x_set_object(radio_param_t param, const void *src, size_t size) {
  return RADIO_RESULT_OK;
}

#define CCA_CLEAR 1
#define CCA_BUSY 0
int sx127x_clear_channel_assesment(){
  return CCA_CLEAR;
}

int sx127x_reset() {
  GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(SX127X_RESET_GPIO_PORT), GPIO_PIN_MASK(SX127X_RESET_GPIO));
  GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(SX127X_RESET_GPIO_PORT), GPIO_PIN_MASK(SX127X_RESET_GPIO));
  GPIO_CLR_PIN(PIN_TO_PORT_BASE(SX127X_RESET_GPIO_PORT), GPIO_PIN_MASK((SX127X_RESET_GPIO)));
  clock_delay_usec(1000);
  ioc_set_over(SX127X_RESET_GPIO_PORT, SX127X_RESET_GPIO, IOC_OVERRIDE_OE);
  GPIO_SET_PIN(GPIO_PORT_TO_BASE(SX127X_RESET_GPIO_PORT), GPIO_PIN_MASK(SX127X_RESET_GPIO));
  clock_delay_usec(110);
  ioc_set_over(SX127X_RESET_GPIO_PORT, SX127X_RESET_GPIO, IOC_OVERRIDE_OE);
  GPIO_CLR_PIN(GPIO_PORT_TO_BASE(SX127X_RESET_GPIO_PORT), GPIO_PIN_MASK(SX127X_RESET_GPIO));
  ioc_set_over(SX127X_RESET_GPIO_PORT, SX127X_RESET_GPIO, IOC_OVERRIDE_PUE);
  clock_delay_usec(5500);

  return 0;
}

int sx127x_initialization() {
  LOG_DBG("Init SPI\n");
  spi_acquire(SX127X_DEV.spi);

  uint8_t ver;
  if ((ver = sx127x_get_version(&SX127X_DEV)) != SX127X_VERSION) {
    LOG_ERR("Error communicating with the module through SPI (invalid version: %d)\n", ver);
    spi_release(SX127X_DEV.spi);
    return RADIO_RESULT_ERROR;
  }

  LOG_DBG("Reset Module\n");
  sx127x_reset();

  sx127x_set_opmode(&SX127X_DEV, sx127x_mode_sleep);

  sx127x_init(&SX127X_DEV);

#if !SX127X_BUSY_RX
  GPIO_SOFTWARE_CONTROL(SX127X_DIO0_PORT_BASE, SX127X_DIO0_PIN_MASK);
  GPIO_SET_INPUT(SX127X_DIO0_PORT_BASE, SX127X_DIO0_PIN_MASK);
  GPIO_DETECT_EDGE(SX127X_DIO0_PORT_BASE, SX127X_DIO0_PIN_MASK);
  GPIO_DETECT_RISING(SX127X_DIO0_PORT_BASE, SX127X_DIO0_PIN_MASK);
  GPIO_TRIGGER_SINGLE_EDGE(SX127X_DIO0_PORT_BASE, SX127X_DIO0_PIN_MASK);
  ioc_set_over(SX127X_DIO0_PORT, SX127X_DIO0_PIN, IOC_OVERRIDE_DIS);
  gpio_hal_register_handler(&sx127x_event_handler_dio0);
  GPIO_ENABLE_INTERRUPT(SX127X_DIO0_PORT_BASE, SX127X_DIO0_PIN_MASK);

  GPIO_SOFTWARE_CONTROL(SX127X_DIO3_PORT_BASE, SX127X_DIO3_PIN_MASK);
  GPIO_SET_INPUT(SX127X_DIO3_PORT_BASE, SX127X_DIO3_PIN_MASK);
  GPIO_DETECT_EDGE(SX127X_DIO3_PORT_BASE, SX127X_DIO3_PIN_MASK);
  GPIO_DETECT_RISING(SX127X_DIO3_PORT_BASE, SX127X_DIO3_PIN_MASK);
  GPIO_TRIGGER_SINGLE_EDGE(SX127X_DIO3_PORT_BASE, SX127X_DIO3_PIN_MASK);
  ioc_set_over(SX127X_DIO3_PORT, SX127X_DIO3_PIN, IOC_OVERRIDE_DIS);
  gpio_hal_register_handler(&sx127x_event_handler_dio3);
  GPIO_ENABLE_INTERRUPT(SX127X_DIO3_PORT_BASE, SX127X_DIO3_PIN_MASK);

  NVIC_EnableIRQ(GPIO_B_IRQn);
#endif

  LOG_INFO("Initialized LoRa module (ver: %d) with SF: %d, CR: %d, BW: %d, CRC: %d, PRLEN: %ld, HEADER: %d\n", 
      ver, 
      SX127X_DEV.lora.sf, 
      SX127X_DEV.lora.cr, 
      SX127X_DEV.lora.bw, 
      SX127X_DEV.lora.crc,
      SX127X_DEV.lora.prlen,
      SX127X_DEV.lora.implicit_header
  );
  LOG_INFO("LoRa driver working with busy RX: %d and interrupt: %d\n", SX127X_BUSY_RX, SX127X_USE_INTERRUPT);
#ifdef MAC_CONF_WITH_TSCH
  LOG_INFO("TX_OFFSET: %d\n", SX127X_TSCH_DEFAULT_TS_TX_OFFSET);
  LOG_INFO("RX_OFFSET: %d\n", SX127X_TSCH_DEFAULT_TS_RX_OFFSET);
  LOG_INFO("TX_ACK_DELAY: %d\n", SX127X_TSCH_DEFAULT_TS_TX_ACK_DELAY);
  LOG_INFO("RX_ACK_DELAY: %d\n", SX127X_TSCH_DEFAULT_TS_RX_ACK_DELAY);
  LOG_INFO("MAX_TX: %d\n", SX127X_TSCH_DEFAULT_TS_MAX_TX);
  LOG_INFO("MAX_ACK: %d\n", SX127X_TSCH_DEFAULT_TS_MAX_ACK);
#endif


  return RADIO_RESULT_OK;
}

const struct radio_driver sx127x_radio_driver = {
  sx127x_initialization,
  sx127x_prepare,
  sx127x_transmit,
  sx127x_send,
  sx127x_read_packet,
  sx127x_clear_channel_assesment,
  sx127x_receiving_packet,
  sx127x_pending_packet,
  sx127x_on,
  sx127x_off,
  sx127x_get_value,
  sx127x_set_value,
  sx127x_get_object,
  sx127x_set_object,
};
