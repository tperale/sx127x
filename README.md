# SX127X LoRa transceiver driver for [contiki-ng](https://github.com/contiki-ng/contiki-ng) OS

This driver was developped as a means of studying the effectiveness of the [TSCH](https://tools.ietf.org/html/rfc7554) protocol in conjunction with LoRa.

## Install

This driver is aimed to be included as a submodule of your *contiki-ng* project and must be refered in your project `Makefile`.

Run the following command in the folder where you want to include the driver.

```
git submodule add https://github.com/tperale/sx127x/
```

## How to use

If you add this driver in the `/arch/dev/` folder of *contiki-ng* (where all the device driver should be) as a submodule, 
add the following line to your `Makefile`.

```
MODULES += $(CONTIKI_NG_DRIVERS_DIR)/sx127x
```

But you can add it whereever you want as long as you refer it from the makefile.

If you are using this driver in conjunction with contiki mac protocol you should make the driver available through the contiki 
NETSTACK and refer it in your `project-conf.h` file.

```
#define NETSTACK_CONF_RADIO sx1272_radio_driver
```

## Working with Zolertia RE-MOTE rev-b

This driver was developped to work with the Zolertia RE-MOTE devboard but should work with any platform supported by contiki-ng.

## TSCH Configuration

To use this driver in conjunction of TSCH more variable should be defined 

```
#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 0, 1, 2 }
extern int tsch_packet_duration(size_t len); 
#define TSCH_PACKET_DURATION(len) tsch_packet_duration(len) 
```

# Credits

* [RIOT-OS](https://github.com/RIOT-OS/RIOT) providing bulletproof SX127X radio driver that served as inspiration while implementing this project.
* [contiki-ng](https://github.com/contiki-ng/contiki-ng)
* [contiki-ng-lora](https://github.com/dtu-ese/contiki-ng-lora) project following the same goal as mine (test TSCH over LoRa)
