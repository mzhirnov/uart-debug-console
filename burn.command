#!/usr/bin/env bash

avrdude -c usbasp -B 125kHz -p attiny13a -U flash:w:main.hex:i
