Multiple DS18B20 temperature sensors with LCD display, SD card logging, remote control via Serial/USB, real time clock, and up to control an output.

Intended for compilation in Atmel Studio and deployment on a custom ATMega328 based circuit. See source code for notes on use with Arduino IDE.

Various library dependencies not in github yet (will be separate repos). See *.atsln

The idea of this is to have 3 temp sensors: one in a well inside the beer (5 gallon bucket), one inside a ply-wood box (contains a heater pad and the bucket), and one in the outside room. Logging temperature for all three will give an idea of the thermal properties of the assembly and of the heat produced by fermentation. In the first instance, a very simple control rule will be used, taking only the temperature from in the wort and testing for a min temperature with fixed hysteresis. See the MS Word Doc.
