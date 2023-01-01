# LoRa_Temp_Humidity
Outdoor temperature and humidity sensor with LoRa transmitter
The microcontroller wakes up once per minute and takes a temperature and relative humidity reading.
It then transmits that data as a 50 byte packet using the LoRa transmitter module (RFM95W).
The data is received by the LoRa receiver on the Raspberry pi.
All calibration is done at the receiver end.
This does NOT use LoRaWAN.
Schematic and PCB are designed using Design Spark.
Enclosure was designed using Fusion 360.
Software was written in C using MPLab X with XC8 compiler.
