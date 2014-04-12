/* ===================================================================
* ****              TempController RTC LCD SD                    ****
*
* Multiple DS18B20 temperature sensors with LCD display, SD card
* logging, remote control via Serial/USB, real time clock, and up to
* control an output (2 outputs in principle but only 1 used).
* ===================================================================
*
* Copyright 2014 Adam Cooper
*
* This is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this software.  If not, see <http://www.gnu.org/licenses/>.
*/

// !!!!!!!!!!! this version uses SdFat library from http://code.google.com/p/beta-lib/downloads/list
// specifically SdFatBeta20130207.zip in an effort to reduce SRAM use, which causes crashing due to overflow
// with the normal Arduino SD library

#include <Arduino.h>
//the following libraries include .h from the .cpp so include .cpp!
#include <MCP7940N.cpp>
#include <I2CUtils.cpp>
#include <EEPROMUtils.cpp>
#include <Wire.cpp>
#include <EEPROM.cpp>
//#include <SD.cpp>
#include <LiquidCrystal.cpp>
#include <OneWire.cpp>
#include <SdFat.cpp> //new lib


//deeper includes, dependencies for SD and Wire
#include <twi.c>
//#include <Sd2Card.cpp>
//#include <SdFile.cpp>
//#include <SdFat.h>
//#include <SdFatUtil.h>
//#include <SdVolume.cpp>
//#include <File.cpp>
#include <SdFile.cpp>
#include <SdBaseFile.cpp>
#include <SdVolume.cpp>
#include <Sd2Card.cpp>

// Function prototypes go here (telling the compiler these functions exist).
void setup();
void loop();

void reportId(uint8_t sensorId[8]);
void saveParamsToEEPROM();
void loadParamsFromEEPROM();
void detectAttachedSensors();
void readSensors();
void stringT(float T, char str2[6]);
void logData();
void controlOutputs();
void readRTC();
void doSerialCommands();
int encodeCommand(char cmd[5]);
void buildLogString();
void sdConnect();

/* Below this point, should be what you need for an Arduino IDE sketch (+ the necessary library includes) */

//these lines are to capture MCUSR as intercepted by optiboot before it resets it.
uint8_t resetFlags __attribute__ ((section(".noinit")));
void resetFlagsInit(void) __attribute__ ((naked)) __attribute__ ((section (".init0")));
void resetFlagsInit(void)
{
	// save the reset flags passed from the bootloader
	__asm__ __volatile__ ("mov %0, r2\n" : "=r" (resetFlags) :);
}

//#define DEBUG //comment out to suppress trace output to Serial
#define WATCHDOG //comment out to suppress use of the watchdog timer, which will cause a reset in the event of a hang (which seems to be associated with powering from a wall wart)

/* Pin definition */
// I2C is (hardware) fixed as A4=SDA, A5=SCL
// SD Card uses SPI and the default SS so uses pins 10-13 (determined by hardware)
// LCD
const int lcdRS = 8;
const int lcdE = 9;
const int lcdD4 =A0;
const int lcdD5 = A1;
const int lcdD6 = A2;
const int lcdD7 = A3;
// pin 2 is for the hardware SD card switch, which triggers an interrupt. May also be configured via solder jumpers for
// - an extra push button input or an interrupt driven from MFP pin
const int sdSwitch = 2;
// user controls
const int modeSwitchPin = 6;//high for run mode, low for setting-mode (USB and button control)
const int pushSwitchPin = 7;//user pulsable button
//outputs for relay control
const int outAPin = 3;
const int outBPin = 4; //output B is not currently used
// 1-wire temp sensors
const int sensorPin = 5;   //a 4.7k pull-up resistor is necessary

#define RUN_MODE HIGH// for reading modeSwitch

#define MAX_1WIRE_SENSORS 3 //max number of 1-wire sensors. Large values may exceed available memory!!!
#define MAX_FIELD_LENGTH 26+6*MAX_1WIRE_SENSORS //max length of a record in logged data, YYYY-MM-DD,HH:MM:SS,A,B,status{,TTT.T}*MAX_1WIRE_SENSORS

/* used to decode commands into an integer */
#define CMD_ECHO 0 //echo test
#define CMD_GETD 1 //get date string
#define CMD_SETD 2 //set date using YYYY-MM-DD string
#define CMD_GETT 3 //get time string
#define CMD_SETT 4 //set time using HH:MM:SS string
#define CMD_LOGN 5 //change name of log file. Must be no longer than 8.3 and must be at least 1.3 (e.g. a.csv)
#define CMD_LOGI 6 //change logging interval in seconds. this is determined by arduino function millis() not the RTC. An integer
#define CMD_TEMP 7//read all attached temp sensors and emit logging string. Forces a check of the control conditions and change of output
#define CMD_TMIN 8//change low temperature threshold for controller. output turns on if T less than this value. May be a float. Maybe negative
#define CMD_THYS 9//change temperature hysteresis. output turns off if T >= TMIN+THYS. May be a float
#define CMD_OUTI 10 //change interval in seconds between controller deciding whether to switch on or off (according to TMIN and THYS). An integer
#define CMD_OUTS 11 //read the output status over serial
#define CMD_RELD 12 //clear stored values in EEPROM. This stops the device and must be followed by a hard reset.
#define CMD_SWAP 13 //swap temp sensor IDs in EEPROM and re-scan. Argument is the single digit sensor index (1,2...) to swap with the sensor ID at index=0.
#define CMD_SCAN 14 //does a rescan of attached sensors and reports the IDs via Serial


//the number of characters expected in the parameter following a command.
//if there are fewer characters available, an error will be signalled and the command ignored
#define MAX_PARAM_LENGTH 12 //the max paramLength
uint8_t paramLength[15];

#define EEPROM_1WIRE_BASE 0x120//address of storage for up to MAX_1WIRE_SENSORS sets of 8 byte (64bit) device IDs, incl CRC
#define EEPROM_PARAMS_SIZE 0x20 //allow 32 bytes for stored params (not all used)
#define EEPROM_PARAMS_BASE 0x100 // 24 bytes for logFileName[12], Tmin, Thys, logInterval, controlInterval

// global variables
//lcdPresent may become true in setup(). Defaults to false => the LCD is not connected
//If LCD not used then a pull-down on LCD E should be inserted instead (if it is desired for the software to skip LCD code chunks)
bool lcdPresent = false;
LiquidCrystal lcd(lcdRS, lcdE, lcdD4, lcdD5, lcdD6, lcdD7);// initialize the LCD library
OneWire  oneWire(sensorPin);
MCP7940N rtc;//The real time clock
SdFat sd;//new lib
SdFile myFile;//new lib
boolean hasSD = false; //is there an SD card
volatile boolean sdInit = false; //whether an SD card initialise is needed on loop. May happen if hot insertion.
boolean sensorsFound=false;
float T[MAX_1WIRE_SENSORS];//for temperature readings
uint8_t controlSensorIndex = 0xFF;//stores the index into T[] to be used as the control temperature. Will be the first detected sensor that is connected at start-up
uint8_t numStoredSensorIds=0;//how many sensor ids stored in EEPROM
uint8_t connectedSensorIds[MAX_1WIRE_SENSORS][8];//holds the sensors actually found against their index number (or a not-found placeholder)

char logString[MAX_FIELD_LENGTH+1]="";
char time[9]=""; //note fixed length format HH:MM:SS + null termination (needed for sprintf)
char date[11]="";//note fixed length format YYYY-MM-DD
boolean runMode = true;
boolean lastRunMode = true;
long lastLogAt = 0;//store value of millis() when last record was logged
long lastControlAt = 0;//store value of millis() when control conditions were last checked
//outputs are turned off initially
boolean outA=false;
boolean outB= false;
//flags logged to last column
//typically the value of bits 3-0 of the MCU status register after startup, when bits 7-4 are set to 1111
uint8_t status =0;

// default values, may be changed according to EEPROM or commands via USB
char logFileName[13] = "log.csv";//as LOGN
unsigned long logInterval=10;// as LOGI
float Tmin = 18.0;// as TMIN
float Thys = 1.0;//as THYS
unsigned long controlInterval = 30;//as OUTI

void setup(){
	//set the status on a reset. Unfortunatly, optiboot boot loader resets MCUSR so this will always set 
	// status to 0xF0, whatever the reset cause but the older arduino bootloader preserves it.
	// A later version of optiboot is believed to pass the value of MCUSR in a register. This is coded for but
	// has not been tested (optiboot version not high enough).
	// Alternatively, just program over the bootloader!
	status=0xF0 | resetFlags;
	
	//enable the watchdog timer with an 8s timeout
	#ifdef WATCHDOG
	cli();//disable interrupts
	__asm__ __volatile__ ("wdr");//reset watchdog timer
	/* Start timed equence */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Set new prescaler(time-out) value = 1024K cycles (~8 s) */
	WDTCSR = (1<<WDE) | (1<<WDP3) | (1<<WDP0);
	sei();// enable_interrupt
	#endif
	
	//the min param lengths
	paramLength[CMD_ECHO]=0;
	paramLength[CMD_GETD]=0;
	paramLength[CMD_SETD]=10;
	paramLength[CMD_GETT]=0;
	paramLength[CMD_SETT]=8;
	paramLength[CMD_LOGN]=4;
	paramLength[CMD_LOGI]=1;
	paramLength[CMD_TEMP]=0;
	paramLength[CMD_TMIN]=1;
	paramLength[CMD_THYS]=1;
	paramLength[CMD_OUTI]=1;
	paramLength[CMD_OUTS]=0;
	paramLength[CMD_RELD]=0;
	paramLength[CMD_SWAP]=1;
	paramLength[CMD_SCAN]=0;

	// set the conventional IO pins. NB pull-ups
	pinMode(modeSwitchPin,INPUT_PULLUP);
	pinMode(pushSwitchPin,INPUT_PULLUP);
	pinMode(outAPin, OUTPUT);
	pinMode(outBPin, OUTPUT);
	pinMode(10, OUTPUT); //SPI Slave-Select
	
	// set up the SD card switch as an interrupt. Pin2 is interrupt 0. Only trigger on edge
	pinMode(sdSwitch,INPUT_PULLUP);
	attachInterrupt(0, sdConnect, CHANGE);//FALLING
	
	//is LCD connected. If LCD not used then a pull-down on LCD E should be inserted instead (if it is desired for the software to skip LCD code chunks)
	pinMode(lcdE, INPUT_PULLUP);
	lcdPresent = digitalRead(lcdE);

	// initialise libraries
	Serial.begin(9600); //Will be outputting to serial
	Wire.begin();//join bus as a master NB this is needed since MCP7940N uses I2C
	rtc.initialise();//initialise the RTC with defaults
	if( lcdPresent){
		pinMode(lcdE, OUTPUT);
		// set up the LCD's number of columns and rows:
		lcd.begin(16, 2);
		// Print a message to the LCD.
		lcd.print("Starting!");
	}
	
	//indicate startup to the user via the OUTB LED
	digitalWrite(outBPin, HIGH);
	delay(500);
	digitalWrite(outBPin, LOW);
	
	// try to initialise SD card.
	// alt use sd.begin(chipSelect, SPI_HALF_SPEED)
	hasSD=sd.begin();//was SD.begin(10)
	if(!hasSD){
		Serial.println("No SD card");
		//alt sd.initErrorPrint()
	}
	
	//if pushbutton is held (active low) on startup then clear stored temp sensor IDs prior to detecting attached sensors
	//if(!digitalRead(pushSwitchPin)){
		//EEPROMUtils::clearBlock(EEPROM_1WIRE_BASE, MAX_1WIRE_SENSORS*8);
		//while(digitalRead(pushSwitchPin));//wait for release since the button is used in set mode inside loop()
	//}

	detectAttachedSensors();

	// check RTC found
	#ifdef DEBUG
	rtc.dumpRegisters();
	#endif

	// setup MFP to trigger interrupt
	rtc.mfpOsc(MCP7940N_RS_1);//set MFP to output 1Hz

	// check EEPROM for operating parameters.
	//If the first byte is 0xFF there are none saved, so save the defaults above. Otherwise load values
	if(EEPROM.read(EEPROM_PARAMS_BASE)==0xFF){
		saveParamsToEEPROM();
	}
	else{
		loadParamsFromEEPROM();
	}
	#ifdef DEBUG
	//commented out because RAM is tight
	//Serial.print("logInterval=");
	Serial.println(logInterval);
	//Serial.print("controlInterval=");
	Serial.println(controlInterval);
	//Serial.print("Tmin=");
	Serial.println(Tmin);
	//Serial.print("Thys=");
	Serial.println(Thys);
	//Serial.print("logFileName=");
	Serial.println(logFileName);
	#endif

	if(  lcdPresent){
		lcd.clear();
	}
}

/* NOTES:
1. changing settings using Serial/USB or manual button-pressing (for Tmin) will not save the values to EEPROM
until re-entering run mode. */
void loop(){
	
	#ifdef WATCHDOG
	 __asm__ __volatile__ ("wdr");//reset watchdog timer
	#endif
	
	//is an SD card re-initialise required? This will be the case if an interrupt was triggered by a SD card hot insertion
	if(sdInit){
		hasSD=sd.begin();
		if(!hasSD){
			lcd.clear();
			lcd.print("SD Card Error");
			delay(1000);
			lcd.clear();
		}
		sdInit=false;
	}
	
	//first check whether run mode or set mode.
	runMode = (digitalRead(modeSwitchPin) == RUN_MODE);
	
	//single-shot special treatment when the run mode changes
	if(runMode!=lastRunMode){
		//save any changed parameters to EEPROM when leaving set mode
		if(runMode){
			saveParamsToEEPROM();
			//turn off outB (setup mode warning)
			digitalWrite(outBPin, LOW);
		}else{			
			//turn off outB (setup mode warning)
			digitalWrite(outBPin, HIGH);
		}
		
		//Show change of run mode on LCD
		if(  lcdPresent){
			lcd.clear();
			lcd.print(runMode?"Run Mode":"Set Mode");
			delay(1000);
			lcd.clear();
		}
		
		lastRunMode = runMode;
	}

	if(runMode){
		if(sensorsFound){
			//check whether the control or logging interval has been met.
			unsigned long t= millis();
			boolean needControl=(t-lastControlAt >= controlInterval*1000);
			boolean needLog=(t-lastLogAt >= logInterval*1000);
			//only read temp once for either control  or logging but always read if LCD present
			if(needControl || needLog ||  lcdPresent){
				//read real time clock first.
				readRTC();
				readSensors();
			}
			//check whether the output should change, including over-shoot alert
			if(needControl){
				lastControlAt = millis();
				controlOutputs();
			}
			//logging
			if(needLog){
				lastLogAt = millis();
				logData();
			}
			//check if button held to request all sensor temperature values are cycled
			while(!digitalRead(pushSwitchPin)){
				if(lcdPresent){
					lcd.clear();
					char sT[6];
					//loop over all stored sensors and check which were found
					for(uint8_t i=0;i<numStoredSensorIds;i++){
						if(connectedSensorIds[i][0] != 0xFF){
							//if sensor connected then emit the ID on the first row of the LCD
							// followed by the temp on the second row
							lcd.setCursor(0, 0);//row=0 col=0
							for(uint8_t j=0; j<8; j++){
								lcd.print(connectedSensorIds[i][j], HEX);
							}							
							lcd.setCursor(0, 1);//row=1 col=0
							lcd.print("Temp=");
							stringT(T[i],sT);
							lcd.print(sT);
							delay(1000);
						}
					}		
					lcd.clear();
				}				
			}
			//update LCD for run mode
			if(  lcdPresent){
				char strTc[6];
				stringT(T[controlSensorIndex], strTc);
				lcd.setCursor(0, 0);//row=0 col=0
				lcd.print("Temp=");
				lcd.print(strTc);
				lcd.print(outA?" ON ":" OFF");
				lcd.setCursor(0, 1);//row=1 col=0
				lcd.print("Status: waiting");
				lcd.setCursor(8, 1);
				if(needLog && hasSD){
					lcd.print("log    ");
				}
				if(needControl){
					lcd.print("ctrl   ");
				}
				//if(!(needControl || needLog)){
					//lcd.print("waiting");
				//}
				//lcd.print("     ");//clears any previous chars
			}
			}else{
			if(  lcdPresent){
				//report no sensors to LCD
				lcd.setCursor(0, 0);//row=0 col=0
				lcd.print("No sensors!");
			}
			
		}//end if(runmode)
	}
	else{
		//check for commands from Serial and execute them accordingly
		doSerialCommands();
		
		//check for button held to change threshold
		if(!digitalRead(pushSwitchPin)){
			Tmin = 0.5+Tmin;
			if(Tmin>30.0){
				Tmin=0.0;
			}
		}
		
		//update LCD for set mode
		if(  lcdPresent){
			char strTmin[6], strThys[5];
			stringT(Tmin, strTmin);
			int iT = (int)Thys;
			int dT = (int)(10*(Thys-(float)Thys));
			sprintf(strThys,"%2d.%1d",iT,dT);
			lcd.setCursor(0, 0);//row=0 col=0
			lcd.print("Min=");
			lcd.print(strTmin);
			lcd.print(" H=");
			lcd.print(strThys);
			lcd.setCursor(0, 1);//row=1 col=0
			lcd.print(date);
			lcd.print(' ');
			lcd.print(time);
		}
	}

	delay(300);
}
/* ===================================
*       Misc Functions
* ====================================*/
//ISR - set a flag for use in loop(), also disable SD writing.
// Writing will be promptly re-enabled only if this was an insertion
void sdConnect(){
	sdInit = (digitalRead(sdSwitch)==LOW);
	hasSD = false;
}

/* ====================================
*           EEPROM Functions
* ====================================*/
void saveParamsToEEPROM(){
	word addr=EEPROM_PARAMS_BASE;
	EEPROMUtils::saveLong(&addr, logInterval);
	EEPROMUtils::saveLong(&addr, controlInterval);
	EEPROMUtils::saveFloat(&addr, Tmin);
	EEPROMUtils::saveFloat(&addr, Thys);
	uint8_t lf[13];
	memcpy(lf,logFileName,13);
	EEPROMUtils::saveBytes(&addr, lf,13);
}

void loadParamsFromEEPROM(){
	word addr=EEPROM_PARAMS_BASE;
	logInterval = EEPROMUtils::loadLong(&addr);
	controlInterval = EEPROMUtils::loadLong(&addr);
	Tmin = EEPROMUtils::loadFloat(&addr);
	Thys = EEPROMUtils::loadFloat(&addr);
	uint8_t lf[13];
	EEPROMUtils::loadBytes(&addr, lf,13);
	memcpy(logFileName,lf,13);
}

/* ====================================
*    Temperature Sensor Functions
* ====================================*/
void reportId(uint8_t sensorId[8]){
	for(uint8_t i = 0; i < 8; i++) {
		Serial.write(' ');
		Serial.print(sensorId[i], HEX);
	}
}

void detectAttachedSensors(){
	uint8_t sensorId[8];
	// temperature sensors have 64 bit IDs but these are used internally and matched to a zero-based index.
	// a lookup table is stored in EEPROM from EEPROM_1WIRE_BASE address, where the 1st stored ID is "sensor 0" etc
	// on first startup, discovered sensors have their IDs stored to EEPROM in the order they are found by OneWire.
	// in future the same sensor will have the same zero-based index
	// so to know up-front which probe will be 0, 1, 2 etc you should connect one probe at a time with a reset/restart
	// to cause it to be detected
	// a look-up array is kept to record which sensors are actually attached (some may be disconnected but retain a record in the EEPROM)

	controlSensorIndex = 0xFF;
	numStoredSensorIds=0;//this does need setting because CMDs SCAN and SWAP call this fn
	
	// see if EEPROM contains sensor IDs. These will be detected from a valid CRC in the 8th stored byte
	uint8_t storedSensorIds[MAX_1WIRE_SENSORS][8];
	word eepromAddress = EEPROM_1WIRE_BASE;
	Serial.println("IDs in EEPROM:");
	while (numStoredSensorIds<MAX_1WIRE_SENSORS){
		EEPROMUtils::loadBytes(&eepromAddress,sensorId,8);
		if(OneWire::crc8(sensorId, 7) != sensorId[7]){
			eepromAddress -=8;//rewind so that newly discovered sensor ids are stored at the right posn
			break;
		}
		reportId(sensorId);
		Serial.println();
		for(uint8_t j = 0; j<8; j++){
			storedSensorIds[numStoredSensorIds][j] = sensorId[j];
		}
		numStoredSensorIds++;
	}
	
	//look for connected temperature sensors.
	//check if their ID is already stored in EEPROM.
	//If not, append the ID to EEPROM list
	//Either way we then know which index it is so save the ID against that index in connectedSensorIds
	for(uint8_t i=0;i<MAX_1WIRE_SENSORS;i++){
		connectedSensorIds[i][0] = 0xFF;//signifies no sensor with index i was found
		T[i]=0.0;//default temp is 0.0 for any sensor that is not present. This should never be used but may appear as a place-holder in output data
	}
	
	Serial.println();
	Serial.println("Detected: ");
	oneWire.reset_search();
	while (oneWire.search(sensorId)) {
		//report what found
		reportId(sensorId);

		//CRC must be good (if bad a bus error)
		if (OneWire::crc8(sensorId, 7) == sensorId[7]) {
			boolean found=false;
			//check EEPROM list
			for(int i=0; i<numStoredSensorIds;i++){
				if(memcmp(storedSensorIds[i],sensorId,8)==0){ //memcmp returns 0 if they match
					//already known
					for(uint8_t j = 0; j<8; j++){
						connectedSensorIds[i][j] = sensorId[j];
					}
					sensorsFound=true;
					Serial.print(" i=");
					Serial.println(i);
					//the first detected sensor will be used for the control temp unless the sensor at index 0 in the EEPROM is present (when it is used)
					if(controlSensorIndex==0xFF) controlSensorIndex = i;
					if(i==0) controlSensorIndex = 0;

					found= true;
					break;
				}
			}
			if(!found && numStoredSensorIds<MAX_1WIRE_SENSORS){
				//not known so add to EEPROM stored sensor list
				//unless wrong type
				if(sensorId[0]==0x28){
					Serial.print(" i=");
					Serial.print(numStoredSensorIds);
					Serial.println(" *new");

					EEPROMUtils::saveBytes(&eepromAddress, sensorId, 8);
					for(uint8_t j = 0; j<8; j++){
						storedSensorIds[numStoredSensorIds][j] = sensorId[j];
						connectedSensorIds[numStoredSensorIds][j] = sensorId[j];
					}
					sensorsFound=true;
					if(controlSensorIndex==0xFF){
						controlSensorIndex = numStoredSensorIds;
					}
					numStoredSensorIds++;
					}else{
					#ifdef DEBUG
					Serial.println("Not DS18B20!");
					#endif
				}
			}
			}else{
			#ifdef DEBUG
			Serial.println("CRC bad!");
			#endif
		}
	}

	//set precision to 11 bit (0.125Celcius) => 375ms max conversion time
	if(sensorsFound){
		//check each index position, 0xFF denotes no sensor found
		for(uint8_t i=0;i<numStoredSensorIds;i++){
			if(connectedSensorIds[i][0] != 0xFF){
				for(uint8_t j=0; j<8; j++){
					sensorId[j] = connectedSensorIds[i][j];
				}
				oneWire.reset();
				oneWire.select(sensorId);
				oneWire.write(0x4E);//write to scratchpad (3 bytes always)
				oneWire.write(0x00);//TH not used
				oneWire.write(0x00);//TL not used
				oneWire.write(0x5F);//config byte
			}
		}
		}else{
		Serial.println("No sensors");
	}
}

// does nothing if there are no sensors
void readSensors(){
	if(sensorsFound){
		byte data[9];
		byte sensorId[8];

		//trigger each sensor to read temperature (takes 375ms at 11 bit precision)
		//then wait and read all sensors. This means only 1 delay is needed
		//check each index position, 0xFF denotes no sensor found
		for(uint8_t i=0;i<numStoredSensorIds;i++){
			if(connectedSensorIds[i][0] != 0xFF){
				for(uint8_t j=0; j<8; j++){
					sensorId[j] = connectedSensorIds[i][j];
				}
				oneWire.reset();
				oneWire.select(sensorId);
				oneWire.write(0x44);//start A-D conversion.
			}
		}
		delay(375);
		for(uint8_t i=0;i<numStoredSensorIds;i++){
			if(connectedSensorIds[i][0] != 0xFF){
				for(uint8_t j=0; j<8; j++){
					sensorId[j] = connectedSensorIds[i][j];
				}
				oneWire.reset();
				oneWire.select(sensorId);
				oneWire.write(0xBE);         // Read Scratchpad command
				for (uint8_t b = 0; b < 9; b++) {           // we need 9 bytes to get the CRC (actually temp is given in 1st two
					data[b] = oneWire.read();
				}
				if(OneWire::crc8(data, 8)){
					// Convert the data to actual temperature
					// because the result is a 16 bit signed integer, it should be stored to an "int16_t" type,
					// which is always 16 bits even when compile on a 32 bit processor.
					int16_t raw = (data[1] << 8) | data[0];
					//this next bit reads the resolution from the config byte so it is always correct, even if the resolution was changed previously
					byte cfg = (data[4] & 0x60);
					// at lower res, the low bits are undefined, so let's zero them
					if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
					else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
					else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
					//// default is 12 bit resolution, 750 ms conversion time
					T[i] = (float)raw / 16.0;
				}
				else{
					#ifdef DEBUG
					Serial.print(i);
					Serial.println(" Bad CRC");
					#endif
				}
			}
			else{
				#ifdef DEBUG
				Serial.print(i);
				Serial.println(" not connected");
				#endif
			}
		}
	}
}

//this should normally be used after both reading sensors and controlling outputs (if a control is due)
//		 since the string contains time, temps and output states
void buildLogString(){
	//build the log data string
	char sT[6];
	//date, time and output status
	sprintf(logString,"%s,%s,%1d,%1d,%02x",date,time,outA,outB, status);
	status=0;//reset status flag now it is logged
	//accumulate the temperatures
	for(uint8_t i=0; i<numStoredSensorIds;i++){
		stringT(T[i],sT);
		strcat(logString, ",");
		strcat(logString, sT);
	}
}

//format a temp as TTT.T (where TTT may include a - sign)
void stringT(float T, char str2[6]){
	int iT = (int)T;
	int dT = abs((int)(10*(T-(float)iT)));
	sprintf(str2,"%3d.%1d",iT,dT);
}


void logData(){
	buildLogString();
	// log to SD card if available
	if(hasSD){
		if (myFile.open(logFileName, O_RDWR | O_CREAT | O_AT_END)) {
			myFile.println(logString);
			// close the file:
			myFile.close();
			}else{
			Serial.print("cant open: ");
			Serial.println(logFileName);
		}
		//File dataFile = SD.open(logFileName, FILE_WRITE);
		//// if the file is available, write to it:
		//if (dataFile) {
		//dataFile.println("foo,bar");//logString
		//dataFile.close();
		//Serial.println("log");
		//}
		//// if the file isn't open, pop up an error:
		//else {
		//Serial.print("cant open: ");
		//Serial.println(logFileName);
		//}
	}
	else{
		//fallback to serial
		Serial.println(logString);
	}
}


//determine whether the outputs should change
//only outA is controlled at present, outB is used to alert over-shoot
void controlOutputs(){
	float Tc = T[controlSensorIndex];
	//control
	if(Tc<Tmin) outA=true;
	if(Tc>=(Tmin+Thys)) outA=false;
	digitalWrite(outAPin, outA?HIGH:LOW);
	//alert
	digitalWrite(outBPin, Tc>(Tmin+Thys));
}

//reads date and time into global variables
void readRTC(){
	rtc.getHH_MM_SS(time);
	rtc.getYYYY_MM_DD(date);
}

// =======================================================
// Serial communications/commands functions
// =======================================================

void doSerialCommands(){
	//check for a command from Serial (via USB)
	Serial.setTimeout(200);
	char serialBuff[MAX_PARAM_LENGTH+6];
	uint8_t bytes = Serial.readBytesUntil(0x0D, serialBuff, MAX_PARAM_LENGTH+5);
	//ideally, commands terminate in CR only, but in reality LF may b send also, which leads to a 1 character read the next time around.
	if(bytes==1){
		if(serialBuff[0]==0x0A){
			bytes=0;
		}
	}
	serialBuff[bytes]=0;//null terminate so it becomes a printable string
	
	boolean validCommand=false;
	
	if(bytes>=5){
		//commands must start with "*" followed by four characters for the command then as many characters as the command requires
		//this will flush out any characters preceding "*"
		boolean startCmd=false;
		uint8_t bi=0;
		while(!startCmd){
			startCmd = (serialBuff[bi]=='*');
			bi++;
			//make sure there are enough characters to make up a command (if there were flushed chars, we might not have enough)
			if(bi>=bytes-4) break;
		}
		
		if(startCmd){
			//read the command (4 chars)
			char cmd[5]="";
			for(int i = 0;i<4;i++) {
				cmd[i] = serialBuff[bi];
				bi++;
			}
			cmd[4]=0;//null termination
			
			//process the command code, encoding the code into an integer for switch/case
			int cmdCode = encodeCommand(cmd);
			
			if(cmdCode>=0){
				//now we know the command, check the param is long enough. if not, force the cmdCode to -1 as if cmd was not valid
				if(bi>bytes-paramLength[cmdCode]){
					Serial.println("Param too short");
					cmdCode=-1;
				}
			}
			
			char cmdParam[MAX_PARAM_LENGTH+1];
			cmdParam[0]=0;
			
			//read the param if the code is OK and there were enough param characters
			if(cmdCode>=0){
				if(paramLength[cmdCode]>0){
					uint8_t i=0;
					while(bi<bytes) {
						cmdParam[i] = serialBuff[bi];
						i++;
						bi++;
					}
					cmdParam[i]=0;
					Serial.println(cmdParam);
				}
				validCommand=true;
			}
			bool badFormat=false;
			switch (cmdCode){
				case CMD_ECHO:
				Serial.print("Echo!");
				break;
				case CMD_GETD:
				rtc.getYYYY_MM_DD(date);
				Serial.println(date);
				break;
				case CMD_SETD:
				badFormat=!rtc.setDate(cmdParam);
				break;
				case CMD_GETT:
				rtc.getHH_MM_SS(time);
				Serial.println(time);
				break;
				case CMD_SETT:
				badFormat=!rtc.setTime(cmdParam);
				break;
				case CMD_LOGN:
				memcpy(logFileName,cmdParam,13);
				//saveParamsToEEPROM(); now happens when switching back to run mode
				break;
				case CMD_LOGI:
				{//need braces because declaring a var inside
					long newLogInterval = atol(cmdParam);
					if(newLogInterval == 0L){
						badFormat=true;
					}
					else{
						logInterval = newLogInterval;
						//saveParamsToEEPROM(); now happens when switching back to run mode
					}
					break;
				}
				case CMD_TEMP:
				readRTC();
				readSensors();
				controlOutputs();
				buildLogString();
				Serial.println(logString);
				break;
				case CMD_TMIN:
				{
					float newTmin = atof(cmdParam);
					if(newTmin == 0.0){
						badFormat=true;
					}
					else{
						Tmin = newTmin;
						lastControlAt=0;//force control check
						//saveParamsToEEPROM(); now happens when switching back to run mode
					}
					break;
				}
				case CMD_THYS:
				{
					float newThys = atof(cmdParam);
					if(newThys == 0.0){
						badFormat=true;
					}
					else{
						Thys = newThys;
						lastControlAt=0;//force control check
						//saveParamsToEEPROM(); now happens when switching back to run mode
					}
					break;
				}
				case CMD_OUTI:
				{
					long newControlInterval = atol(cmdParam);
					if(newControlInterval == 0L){
						badFormat=true;
					}
					else{
						controlInterval = newControlInterval;
						//saveParamsToEEPROM(); now happens when switching back to run mode
					}
					break;
				}
				case CMD_OUTS:
				Serial.print("A:");
				Serial.print(outA?"on":"off");
				Serial.print(" B:");
				Serial.print(outB?"on":"off");
				break;
				
				case CMD_RELD:
				EEPROMUtils::clearBlock(EEPROM_1WIRE_BASE, MAX_1WIRE_SENSORS*8);
				EEPROMUtils::clearBlock(EEPROM_PARAMS_BASE, EEPROM_PARAMS_SIZE);
				Serial.println("Cleared. Now reset hardware");
				break;
				
				case CMD_SWAP:
				{
					int offset = atoi(cmdParam)*8;
					if(offset>0){
						uint8_t temp;
						word eepromAddress1;
						word eepromAddress2;
						//copy sensor ID at the index specified as the SWAP parameter to index 0
						for(uint8_t ii=0;ii<8;ii++){
							eepromAddress1 = EEPROM_1WIRE_BASE+ii;
							eepromAddress2 = eepromAddress1 + offset;
							temp = EEPROM.read(eepromAddress1);
							EEPROM.write(eepromAddress1,EEPROM.read(eepromAddress2));
							EEPROM.write(eepromAddress2, temp);
						}
					}
					//re-scan attached sensors. This makes sure the runtime data is updated to match the swap
					detectAttachedSensors();
				}
				break;
				
				case CMD_SCAN:
				EEPROMUtils::clearBlock(EEPROM_1WIRE_BASE, MAX_1WIRE_SENSORS*8);
				detectAttachedSensors();
				break;
				
				default:
				validCommand = false;
			}//end switch
			if(badFormat){
				Serial.print("bad format: ");
				Serial.println(cmdParam);
			}
		}// end startCmd
	}
	if(bytes>0 && !validCommand){
		Serial.print("Bad cmd:");
		Serial.println(serialBuff);
	}
}

//turn a 4 character command into its integer code. use cm
int encodeCommand(char cmd[5]){
	if(!strcmp(cmd,"ECHO")) return CMD_ECHO;
	if(!strcmp(cmd,"GETD")) return CMD_GETD;
	if(!strcmp(cmd,"SETD")) return CMD_SETD;
	if(!strcmp(cmd,"GETT")) return CMD_GETT;
	if(!strcmp(cmd,"SETT")) return CMD_SETT;
	if(!strcmp(cmd,"LOGN")) return CMD_LOGN;
	if(!strcmp(cmd,"LOGI")) return CMD_LOGI;
	if(!strcmp(cmd,"TEMP")) return CMD_TEMP;
	if(!strcmp(cmd,"TMIN")) return CMD_TMIN;
	if(!strcmp(cmd,"THYS")) return CMD_THYS;
	if(!strcmp(cmd,"OUTI")) return CMD_OUTI;
	if(!strcmp(cmd,"OUTS")) return CMD_OUTS;
	if(!strcmp(cmd,"RELD")) return CMD_RELD;
	if(!strcmp(cmd,"SWAP")) return CMD_SWAP;
	if(!strcmp(cmd,"SCAN")) return CMD_SCAN;
	
	return -1;
}