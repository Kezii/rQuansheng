# rQuansheng

This is an highly experimental from-scratch reimplementation for a firmware for the Quansheng UV-K5 radio

This is not a black box reimplementation, I'm still using the C code as reference, but the point is to move away from that, especially on the UI 

# Development

Firmware is in `cross/rquansheng`

PC software in `host_sw`

# Roadmap

## Basics
- [x] run a binary
- [x] run RTIC
- [x] gpio driver
- [ ] adc / battery level
- [x] bk4819 bitbang driver
- [x] bk4819 hal and library
- [x] keyboard driver and events
- [x] display driver
- [ ] eeprom driver 
- [ ] bk4819 hal refactoring (partial)

## High level
- [x] fm radio rx
- [x] fm radio tx
- [ ] serial protocol for remote control
- [ ] defmt logs wrapped by the serial protocol
- [ ] centralized radio state
- [ ] basic UI for radio state
- [ ] basic UI for menu
- [ ] eeprom settings save


## In the far future, probably never
- [ ] AM / SSB
- [ ] full support for CTCSS / CDCSS etc etc
- [ ] spectrum analysis
- [ ] scanning
- [ ] ARDF

![photo](docs/photo.jpg)