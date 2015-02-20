In an effort to alleviate cumbersome wires, we are attempting to use the Nordic SemiCondutor nRF24l01+ modules for 2.4Ghz datatransfer. Using the Rf24 library from maniacbug, we communicate with the nRF24l01+ and can send data at rates of up to 2Mbps. SPI protocol is used to configure and communicate with the transceivers. The ultimate goal of this is to have several Wireless Sensor Packets that communicate with IMUs and flex sensors that communicate in a star formation back to a main base node and feed the sensor information into the main Matlab program.

Wireless Sensor Packet (proposed) Component List
Arduino Pro Mini (3.3V, 8MHz) w/ ATMega328
Sparkfun MPU-9150 Breakout Board
Nordic SemiConductor nRF24l01+ http://www.amazon.com/gp/product/B00E594ZX0/ref=oh_aui_detailpage_o09_s00?ie=UTF8&psc=1 
Polymer Lithium Ion Battery (1000mAh ~ 2000mAh)
SparkFun LiPo Charger Basic - Micro-USB
Sparkfun Flex Sensor 4.5"
Button


Testing
To learn how to use the RF24 library, we found an example of a star formation called starping.pde http://maniacbug.github.io/RF24/starping_8pde-example.html (we copied this into an .ino file). In this setup, the same code can be written to each Arduino participating in the network. The role of the arduino is determined by the status of pin 7. If the value of pin 7 is connected to GND, the arduino behaves as a "pong out" node, which recieves the incoming signals and sends it back to the sender. All other Arduinos behave as "ping" modules which initiate the signal. Each ping module needs an ID. This can be set up in the serial monitor of the Arduino IDE. ID's 2-6 are available to the ping modules, while ID 1 is reserved for the pong back module. An ID of 0 indicates that no ID has been assigned to the node. Once IDs are assigned for a ping module, they are stored in the EEPROM of the Arduino, so they will remain until changed. Slight modification of the starping code is needed for operation on an Arduino Mega due to the specific pin callouts of the SPI functionality on the Mega. Specifically, pin 10 on the Arduino Uno and Pro Mini needs to be changed to pin 53 on the Mega. More information on the setup of the nRF24l01+ can be found on this website: http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo. It is advisable to solder a 10microF or greater electrolytic capacitor across the VCC and GND pins on the nRF24l01 board, especially when using the Mega. This is due to some issue with the power supplied by the Mega which causes rampant packet loss on the nRF24l01+.

Implementation
We have confirmed the basic example of the starping setup. Now we need to merge the IMUQuat and Starping .ino's to acheive full functionality. This is still being worked on.
