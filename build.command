#!/usr/bin/env bash

avr-gcc -std=c++20 -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti \
	-mmcu=attiny13a -mtiny-stack -mshort-calls -maccumulate-args -mrelax \
	-fpack-struct -fshort-enums -ffunction-sections -fdata-sections -Wl,-gc-sections \
	-Os -DF_CPU=9600000 ./main.cpp -o ./main.elf
avr-objcopy --only-section .text --only-section .data -O ihex ./main.elf ./main.hex
avr-size --mcu=attiny13a --format=avr ./main.elf
avr-objdump -d --demangle ./main.elf > ./main.lss