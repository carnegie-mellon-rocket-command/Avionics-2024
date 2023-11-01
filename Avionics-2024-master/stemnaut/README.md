# Required Libraries

- Seeed nRF52 Boards (not a library)
- Seeed Arduino LSM6DS3
- Adafruit_LIS331

and all dependencies.

# Using the sketch

Flash to the Seeed. On normal boot (pin 0 unconnected), it will log data. Short pin 0 to ground to enter read mode, where it will dump data to Serial. For now you can just copy data from serial into a text file and save it as .csv (remove any debugging output), and it should be open-able in say Excel. 
