# PulseOximetryLabVIEW
This simple project aims to test MAX30102 sensor which is an integrated pulse oximetry and heart-rate monitor. 

![FrontPanel](https://github.com/OTRIUM/PulseOximetryLabVIEW/blob/master/FrontPanel.jpg)

STM32 is used to read data from the I2C sensor (max30102) and sned it to a PC via USB.
LabVIEW GUI provides you with: 
+ raw data from both IR and Red LEDs (White and Red curves respectively)
+ calculated heart rate and O2 saturation values
+ ability to change some settings of the sensor

## Requirements
+ MAXREFDES117# Heart-Rate and Pulse-Oximetry Monitor Module
+ STM32F103C8T6 eval board with USB available
+ Windows PC with NI LabVIEW installed
