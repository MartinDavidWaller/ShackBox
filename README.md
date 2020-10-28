# ShackBox

The ShackBox project is documented in Practical Wireless, April 2018. If you require further information,
such as the associated circuit diagram, then please contact me at martin@the-wallers.net.

To build and install this software you will need to:

1) Install the Ardiuno Interactive Development Environment. This
can be found here:

https://www.arduino.cc/en/Main/Software

Once installed you will need to install the following libraries.

   -> Adafruit BMP085 Library by Adafruit 1.0.0

This can be done from the Sketch | Include Library | Manage Libraries...
menu option.

   -> Library for the LiquidCrystal LCD display connected to an Arduino board

This can be downloaded from here:
 
     https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library

The link also provides installation instructions.

   -> Efficient alternative to SoftwareSerial with attachInterrupt for RX chars, simultaneous RX & TX

This can be downloaded from here:

     https://github.com/SlashDevin/NeoSWSerial

It can be installed in a similar fashion to the LCD library

2) Connect the Arduino to your PC using the correct cable.
3) On the Tools | Board menu option select "Arduino Nano" 
4) On the Tools | Port menu option select the correct COM port
    for the connected device.
5) Select the Sketch | Upload menu option. This should compile
    the code and upload it to the Arduino Nano. 

At this point your software is installed. This the circuit is
complete and correctly built then you should see data on the
display.

Fault Finding

The BMP180 and I2C LCD units sit on the I2C bus and each has a unique address. 

The Adafruit BMP085 code assumes an address of 0x77 – see Adafruit_BMP085.h line 30.

The I2C LCD code assumes an address of 0x27 – see ShackBox.ino line 280.

If either of these are incorrect that you will get failure. If the BMP085 address is wrong, 
then the temperature / pressure will probably display as zero. If the I2C LCD address is wrong, 
then you will get no display.

If problems are found, then the code here:

https://playground.arduino.cc/Main/I2cScanner

can be used to scan the bus a derive the correct addresses. It’s probably best to unplug one
of the devices, and the easy one to unplug is the LCD, so remove any confusion about which 
address is being returned. 

The code should then be changed to reflect any new addresses and recompiled.

If you get the spash screen up and nothing more then the GPS unit may be emitting unexpected
talker IDs in the NMEA. It would be worth looking at lines 146 and 155 and changing the $GP
text to $GN.

There has been a report of an uploading issue where the code commpiles but fails to upload
to the Arduino. typically this looks like:

avrdude: stk500_getsync() attempt 10 of 10: not in sync: resp=0x00

It turns out there the are devices out there with 'old' bootloader code and to resolve the issue
you need to select the ATmega328P (old Bootloader) option in the Tools menu of the IDE.



