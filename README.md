# rquansheng

this is an highly experimetal from-scratch reimplementation for a firmnware for the quansheng uv-ke radio

this is still based on the reverse-engineered C firmware, i.e. not a black box reimplementation, but it's a clean source code rewrite


# roadmap

## framework
- [x] run a binary
- [x] run RTIC
- [x] gpio driver
- [ ] adc / battery level
- [x] working bk4819 bitbang driver
- [x] working bk4819 hal and library
- [ ] keyboard driver and events
- [ ] display driver
- [ ] eeprom driver 

## usage
- [x] fm radio rx
- [ ] fm radio tx
- [ ] basic UI
- [ ] eeprom settings save