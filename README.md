# rquansheng

This is an highly experimental from-scratch reimplementation for a firmware for the Quansheng UV-K5 radio

This is not a black box reimplementation, I'm still using the C code as reference, but the point is to move away from that, especially on the UI 


# Roadmap

## Basics
- [x] run a binary
- [x] run RTIC
- [x] gpio driver
- [ ] adc / battery level
- [x] bk4819 bitbang driver
- [x] bk4819 hal and library
- [ ] keyboard driver and events
- [ ] display driver
- [ ] eeprom driver 

## High level
- [x] fm radio rx
- [x] fm radio tx
- [ ] serial protocol for debug logs and remote control
- [ ] basic UI for radio state
- [ ] eeprom settings save
- [ ] basic ui for menu
- [ ] other modes