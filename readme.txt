This is an Arduino-as-Port-Extender Library by Petr Osipov

Installation
--------------------------------------------------------------------------------

1) Download the library as ZIP file. 
2) In your Arduino DevEnv, go to Sketch->Include Libraries->Add .ZIP Library and select the library
3) To prepare the client board, go the \Client folder, open the ArduinoPortExtenderClient sketch, compile it and upload it to the board which would be an I2C client. In that sketch, change #define CLIENT_I2C_ADDRESS 0x22 to the I2C Address you wish for your client.
4) Connect the Master board and the client board(s). Connect all SCL pins together and all SDA pins together. Usual pins see below. Also connect GND of all boards.
5) See an example sketch in examples folder.

I tested it on Arduino Nano, Uno and Mega, but any board supported by Arduino (ESP8266, ESP32, STM32..) should work. You will however have to find out the pin numbers for that board.

Usual I2C Pins
Board	I2C / TWI pins
Uno, Nano, Uno Ethernet - 	A4 (SDA), A5 (SCL)
Mega2560 -	20 (SDA), 21 (SCL)
NodeMCU, Wemos D1R2 - D1(SCL), D2 (SDA) 
STM32 Bluepill - PB6/8(SCL1),PB7/9(SDA1),PB10(SCL2),PB11(SDA2)


