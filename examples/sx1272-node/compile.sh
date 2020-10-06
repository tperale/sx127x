#!/bin/bash
if [ "$1" = "0" ]; then
    make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB$1 node.upload
else
    make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB$1 coordinator.upload
fi
make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB$1 login
