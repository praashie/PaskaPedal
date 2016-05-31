# PaskaPedal

![The inspiration](./pedals.JPG)


PaskaPedal is a small electronics project built on two unused KORG guitar volume pedals,
turning them into a class-compliant(-enough-for-Linux) USB MIDI controller,
used for controlling synthesizers and guitar effects.

This repository contains the firmware source.

![Main board](./board_top.JPG)
### Main components
* Atmel ATTiny84 with 16MHz crystal
* LM317 voltage regulator at ~3.4V
* 2x KORG FK-3 volume pedals with 25kOhm potentiometers

### Features
* Acts as a driverless USB device
* Provides 2 continuous and 2 two-state MIDI CC controllers
* Manual calibration for pedal extremes
* Connection for a guitar amplifier footswitch for additional looper/effect control

## Issues
* Doesn't work on Macs
* Calibration isn't stored in on-board EEPROM

[Demonstration video](https://youtu.be/3chLeBRuCPc)
