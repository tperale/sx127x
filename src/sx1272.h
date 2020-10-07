#ifndef SX1272_H_
#define SX1272_H_

#include "contiki.h"
#include "dev/radio.h"
#include "dev/spi.h"
#include "sx1272-api.h"

#ifdef SX1272_SPI_SCK_PORT_CONF
#define SX1272_SPI_SCK_PORT SX1272_SPI_SCK_PORT_CONF
#else
#define SX1272_SPI_SCK_PORT SPI1_CLK_PORT
#endif
#ifdef SX1272_SPI_MISO_PORT_CONF
#define SX1272_SPI_MISO_PORT SX1272_SPI_MISO_PORT_CONF
#else
#define SX1272_SPI_MISO_PORT SPI1_RX_PORT
#endif
#ifdef SX1272_SPI_MOSI_PORT_CONF
#define SX1272_SPI_MOSI_PORT SX1272_SPI_MOSI_PORT_CONF
#else
#define SX1272_SPI_MOSI_PORT SPI1_TX_PORT
#endif

#ifdef SX1272_SPI_CS_PORT_CONF
#define SX1272_SPI_CS_PORT SX1272_SPI_CS_PORT_CONF
#else
#define SX1272_SPI_CS_PORT GPIO_B_NUM
#endif

#ifdef SX1272_SPI_SCK_CONF
#define SX1272_SPI_SCK SX1272_SPI_SCK_CONF
#else
#define SX1272_SPI_SCK SPI1_CLK_PIN
#endif

#ifdef SX1272_SPI_MISO_CONF
#define SX1272_SPI_MISO SX1272_SPI_MISO_CONF
#else
#define SX1272_SPI_MISO SPI1_RX_PIN
#endif

#ifdef SX1272_SPI_MOSI_CONF
#define SX1272_SPI_MOSI SX1272_SPI_MOSI_CONF
#else
#define SX1272_SPI_MOSI SPI1_TX_PIN
#endif

#ifdef SX1272_SPI_CS_CONF
#define SX1272_SPI_CS SX1272_SPI_CS_CONF
#else
#define SX1272_SPI_CS 6
#endif

#ifdef SX1272_SPI_BITRATE_CONF
#define SX1272_SPI_BITRATE SX1272_SPI_BITRATE_CONF
#else
#define SX1272_SPI_BITRATE 8000000
#endif

#ifdef SX1272_SPI_PHASE_CONF
#define SX1272_SPI_PHASE SX1272_SPI_PHASE_CONF
#else
#define SX1272_SPI_PHASE 0
#endif

#ifdef SX1272_SPI_POL_CONF
#define SX1272_SPI_POL SX1272_SPI_POL_CONF
#else
#define SX1272_SPI_POL 0
#endif

#ifdef SX1272_SPI_CONTROLLER_CONF
#define SX1272_SPI_CONTROLLER SX1272_SPI_CONTROLLER_CONF
#else
#define SX1272_SPI_CONTROLLER 0
#endif

#ifdef SX1272_RESET_GPIO_PORT_CONF
#define SX1272_RESET_GPIO_PORT SX1272_RESEST_GPIO_PORT_CONF
#else
#define SX1272_RESET_GPIO_PORT GPIO_A_NUM
#endif

#ifdef SX1272_RESET_GPIO_CONF
#define SX1272_RESET_GPIO SX1272_RESEST_GPIO_CONF
#else
#define SX1272_RESET_GPIO 5 // CC1200_SPI_CSN_PIN
#endif

#define SX1272_TSCH_DEFAULT_TS_MAX_TX 363776

#define SX1272_TSCH_DEFAULT_TS_CCA_OFFSET 0
#define SX1272_TSCH_DEFAULT_TS_CCA 0
#define SX1272_TSCH_DEFAULT_TS_TX_OFFSET 2500
#define SX1272_TSCH_DEFAULT_TS_RX_OFFSET (SX1272_TSCH_DEFAULT_TS_TX_OFFSET - (TSCH_CONF_RX_WAIT / 2))
#define SX1272_TSCH_DEFAULT_TS_TX_ACK_DELAY 3000
#define SX1272_TSCH_DEFAULT_TS_RX_ACK_DELAY (SX1272_TSCH_DEFAULT_TS_TX_ACK_DELAY - (TSCH_CONF_RX_WAIT / 2))
#define SX1272_TSCH_DEFAULT_TS_RX_WAIT 5000
#define SX1272_TSCH_DEFAULT_TS_ACK_WAIT 5000 // TSCH_CONF_RX_WAIT
#define SX1272_TSCH_DEFAULT_TS_RX_TX 0
#define SX1272_TSCH_DEFAULT_TS_MAX_ACK 82000
#define SX1272_TSCH_DEFAULT_TS_TIMESLOT_LENGTH ( SX1272_TSCH_DEFAULT_TS_TX_OFFSET \
        + SX1272_TSCH_DEFAULT_TS_MAX_TX \
        + SX1272_TSCH_DEFAULT_TS_TX_ACK_DELAY \
        + SX1272_TSCH_DEFAULT_TS_MAX_ACK )

 extern sx1272_t __sx1272_dev;

#ifdef SX1272_DEV_CONF
#define SX1272_DEV SX1272_DEV_CONF
#else
#define SX1272_DEV __sx1272_dev
#endif 

extern sx1272_t __sx1272_dev;
extern const struct radio_driver sx1272_radio_driver;
#endif
