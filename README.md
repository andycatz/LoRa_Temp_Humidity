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

This sensor requires DC 9V power to run the electronics and the fan.  A battery is possible but
it would need to be reasonable large e.g. lead-acid 12V.
The fan is used to pass ambient air over the sensors and to prevent the outer walls of the enclosure
from heating up the sensor.
The 12V fan is run at approximately 5V so that it should have a long life time.  An IP65 or thereabouts
rated fan should be used, otherwise the fan bearings will rust within a few months.
In addition to the outside temperature and relative humidity, the sensor also reports back its own
internal temperature as well as the supply voltage, microcontroller supply voltage and fan voltage.
