.PHONY: build clean flash-surface flash-onboard
build: surface/build/arduino.avr.nano/surface.ino.hex onboard/build/arduino.avr.nano/onboard.ino.hex

surface/build/arduino.avr.nano/surface.ino.hex: surface/surface.ino
	arduino-cli compile -b arduino:avr:nano surface -e
onboard/build/arduino.avr.nano/onboard.ino.hex: onboard/onboard.ino
	arduino-cli compile -b arduino:avr:nano onboard -e

flash-surface: surface/build/arduino.avr.nano/surface.ino.hex
	avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:surface/build/arduino.avr.nano/surface.ino.hex:i
flash-onboard: onboard/build/arduino.avr.nano/onboard.ino.hex
	avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:onboard/build/arduino.avr.nano/onboard.ino.hex:i
clean: build
	rm -r surface/build
	rm -r onboard/build
