Project "Rebel Coffee Scale"

PARTS USED
- Raspberry Pi Pico 2 (also suitable for esp32 devkit and similar) 
- 2 kg load cell (will be 5 kg in the final project)
- Mini560 3.3v (for converting power from batteries)
- NAU7802 - for converting data from the cell
- 2 screens SPI OLED 0.96"
  
- 18650 batteries (2x, will be 14500 in final project)
- 3 "buttons" and 1 power switch

- Fuse 0.5A
- Capacitors: 2 x 100 nF, 2 x 47 uF
- Diode for + (to prevent improper polarity)
- Resistors: 47k, 100k (to read the battery %)

CONCEPT
Batteries will be charged by using a separate charger. Scales should have a nice speed and accuracy. Repeirability and adjustability. 
Key feature - proper auto-stopwatch. And predictable stopwatch which is easy to reset (push - counting - push - reset).
Physical turn off / on switch to not deal with the sensors and to not wait for them to turn off.
