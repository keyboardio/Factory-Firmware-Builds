DEVICE_PORT=/dev/ttyACM1
DEVICE_PORT_BOOTLOADER=/dev/ttyACM1

ARDUINO_PATH=/usr/local/arduino
ARDUINO_LOCAL_LIB_PATH=${HOME}/Arduino
ARDUINO_TOOLS_PATH=${ARDUINO_PATH}/hardware/tools
ARDUINO_BUILDER=${ARDUINO_PATH}/arduino-builder
ARDUINO_IDE_VERSION=100607
BOARD_HARDWARE_PATH=${ARDUINO_LOCAL_LIB_PATH}/hardware
BOARD=model01
MCU=atmega32u4
FQBN=keyboardio:avr:${BOARD}
BUILD_PATH := $(shell mktemp -d 2>/dev/null || mktemp -d -t 'build')

SKETCH=attiny_flasher.ino

build:
	${ARDUINO_BUILDER} -compile \
	            -verbose \
                    -hardware "${ARDUINO_PATH}/hardware" \
                    -hardware "${BOARD_HARDWARE_PATH}" \
                    -tools "${ARDUINO_PATH}/tools-builder" \
		    -tools "${ARDUINO_PATH}/hardware/tools" \
                    -fqbn "${FQBN}" \
                    -libraries "${BOARD_HARDWARE_PATH}/.." \
        	    -libraries "." \
                    -build-path "${BUILD_PATH}" \
                    -ide-version "${ARDUINO_IDE_VERSION}" \
                    -warnings all \
                    -prefs "compiler.cpp.extra_flags=-std=c++11 -Woverloaded-virtual -Wno-unused-parameter -Wno-unused-variable -Wno-ignored-qualifiers" \
                    ${SKETCH}
	cp ${BUILD_PATH}/${SKETCH}.hex .


flash: build
	stty -F `ls /dev/ttyACM*` 1200 hupcl
	sleep 2
	avrdude -p${MCU} -cavr109 -D -P `ls /dev/ttyACM*` -b57600 "-Uflash:w:${SKETCH}.hex:i"

