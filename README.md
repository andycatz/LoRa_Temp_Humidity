# LoRa_Temp_Humidity
Outdoor temperature and humidity sensor with LoRa transmitter
The microcontroller wakes up once per minute and takes a temperature and relative humidity reading.
It then transmits that data as a 50 byte packet using the LoRa transmitter module (RFM95W).
The data is received by the LoRa receiver on the Raspberry pi.
All calibration is done at the receiver end.
This does NOT use LoRaWAN.
