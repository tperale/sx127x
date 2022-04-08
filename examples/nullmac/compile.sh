#!/bin/bash
make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB$1 test-sx1272.upload
make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB$1 login
