/* Farmland Automation Mote: 
 *
 *   Author	: Vignesh Natarajan (a) Viki 
 *   Copyright 	: www.vikilabs.org	
 *   Date: 2015-10-1
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * Circuit Configuration:
 * 
 *  1)	Analog sensors on analog ins 0, 1, and 2
 *  2)	SD card attached to SPI bus as follows:
 *		-> MOSI - pin 11
 *		-> MISO - pin 12
 *		-> CLK - pin 13
 *		-> CS - pin 10
 *
 * This device is based on Atmel 328 Microcontroller and ESP8266 with the following sensors
 * 	-> Soil Temperature
 *	-> Humidity
 *	-> Soil Moisture
 *	-> Soil Conductivity
 * 
 * This device is able to work with 4 AA batteries for 6 months by transmitting sensor data to the server
 * over WiFi at a frequency of 4 transmits per hour.
 */

#include <SPI.h>
#include <SD.h>
#include <DHT.h>
#include <OneWire.h>
#include <Wire.h>
#include <EEPROM.h>
#include "RTClib.h"
#include "SoftwareSerial.h"


#define DEBUG 0
#define SETUP_DEBUG 0
#define PRODUCTION 1
#define DEVEL 0

#define DHT22_PIN 7   // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// Device ID is stored at 10

#define eepromAddr 10

#define SSID "vikilabs"
#define PASS "*******"
#define DST_IP "192.168.1.1" // Server IP address..

/*@Viki: Power Optimization Code for Battery Powered board*/
#define POWER_OPTIMIZATION

//Global Variables
DateTime now;
int BoardID;

DHT dht(DHT22_PIN, DHTTYPE);
RTC_DS1307 RTC;

#define WIFI_ENABLE_PIN 4

OneWire  ds(5);  // on pin 5 (a 4.7K resistor is necessary)
#define chipSelect 10 /*@Viki changine chipSelect to macro to save memory*/

SoftwareSerial Serial2(2, 3); // RX, TX
boolean connected = false;

#ifdef POWER_OPTIMIZATION
#define SLEEP_MULTIPLE_OF_8_SEC(val)\
  for(int ctr=0; ctr<val; ctr++){\
    power_management_goto_sleep();\
  }
#endif 

void PrintFromESP()
{

  while (Serial2.available())
  {
    // The esp has data so display its output to the serial window
    char c = Serial2.read(); // read the next character.
#if DEBUG
    Serial.write(c);
#endif

  }
}


boolean connectWiFi()
{

  Serial2.begin(9600);
  while (!Serial2);


  //test if the module is ready
#if DEBUG
  Serial.println("Resetting module");
#endif

  Serial2.println("AT+RST");
  delay(2000);
  PrintFromESP();

  //Set the client mode..
  Serial2.println("AT+CWMODE=1");
  delay(1000);

  PrintFromESP();

  String cmd = "AT+CWJAP=\"";
  cmd += SSID;
  cmd += "\",\"";
  cmd += PASS;
  cmd += "\"";
#if DEBUG
  Serial.println("Joining to AP");
  Serial.println(">>" + cmd);
#endif
  Serial2.println(cmd);
  delay(3000);

  PrintFromESP();

  if (Serial2.find("CONNECTED"))
  {
#if DEBUG
    Serial.print("+");
    Serial.println("OK, Connected to WiFi.");
    Serial.println(">>AT+CIFSR");
    Serial2.println("AT+CIFSR");
    delay(500);
    PrintFromESP();
#endif
    Serial2.println("AT+CIPMUX=0");
    delay(1000);

    PrintFromESP();
    return true;
  } else
  {
#if DEBUG
    Serial.print("-");
    Serial.println("Can not connect to the WiFi.");
#endif
    return false;
  }

}

boolean disconnectWiFi()
{

  Serial2.begin(9600);
  while (!Serial2);


  //test if the module is ready
#if DEBUG
  Serial.println("Disconnecting WiFi module");
#endif

  Serial2.println("AT+CWQAP");
  delay(1000);
  connected = false;
  PrintFromESP();
}

boolean TCPpost (int soil, int AT, int RH, float celsius, float voltage) 
{
  String cmd;
  String UnxTime;

  cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += DST_IP;
  cmd += "\",8000";

  //Open a TCP channel..
  Serial2.println(cmd);

#if DEBUG
  Serial.println(">>" + cmd);
#endif

  if (Serial2.find("Error")) {
    return false;
  }

  //Not sure what is the issue, int UnxTime = now.unixtime do not work..

  UnxTime = (String)now.unixtime();

  cmd = (String)BoardID 
    + "," + UnxTime
    + "," + soil
    + "," + celsius
    + "," + AT
    + "," + RH 
    + "," + voltage
    + "\n";

  Serial2.print("AT+CIPSEND=");
  Serial2.println(cmd.length());
  delay(100);
  if (Serial2.find(">"))
  {
    //    Serial.print(">");
  } else
  {
    Serial2.println("AT+CIPCLOSE");
    //  Serial.println("connect timeout");
    return false;
  }
#if DEBUG
  Serial.println(">>" + cmd);
#endif
  Serial2.print(cmd);
  delay(1000);
  PrintFromESP();
  Serial2.println("AT+CIPCLOSE");
  PrintFromESP();
  //delay(1000);

  return true;
}


void ValidatePost(int soil, int AT, int RH, float celsius, float voltage) 
{

  for (int i = 0; i < 5; i++) // Make five attempts to transmit
  {
    if(TCPpost (soil, AT, RH, celsius, voltage)) return;
    delay(500); //delay before we make another attempt
  }

#if DEBUG
  Serial.println("Post failed, looks like connection dropped, should attempt again");
#endif

  connected = false;
  return;
}


void setup() 
{

#if DEBUG 
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial);
#endif

  //Read boardID and store

  EEPROM.get(eepromAddr,BoardID);

  pinMode(WIFI_ENABLE_PIN, OUTPUT);
  digitalWrite(WIFI_ENABLE_PIN, HIGH);

#if DEBUG
  Serial.println(BoardID);
  Serial.println("BoardID printed");
#endif

  //*rtc clock
  Wire.begin();
  RTC.begin();

  if (! RTC.isrunning()) {

#if DEBUG||SETUP_DEBUG
    Serial.println("RTC is NOT running!");
#endif
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

#if DEBUG
  Serial.println("Initializing SD card...");
#endif

  if (!SD.begin(chipSelect))
  {

#if DEBUG
    Serial.println("Card failed, or not present");
#endif

    return;
  }
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);

#if DEBUG
  Serial.println("card initialized.");
#endif


  //connect to the wifi
  connected = false;
  for (int i = 0; i < 3; i++) //Three attempts to connect
  {
    if (connectWiFi())
    {
      connected = true;
      break;
    }
  }

#ifdef POWER_OPTIMIZATION
  power_management_init_watchdog_timer();
  power_management_enable_sleep_mode();
#endif

}


void loop()
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  static short count=7; //upload the first data

  /*@Viki: Moving Global to Local to save memory*/
  float celsius;
  int  soil;
  float voltage;
  int AT;
  int RH;
  
  //  float fahrenheit;

  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(25);
    return;
  }

  for ( i = 0; i < 8; i++) {
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
#if DEBUG
    Serial.println("CRC is not valid!");
#endif

    return;
  }

  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      return;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  // fahrenheit = celsius * 1.8 + 32.0;
#if DEBUG
  Serial.print(" S_Temperature = ");
  Serial.print(celsius);
  Serial.print(",");
#endif

  // Serial.print(fahrenheit);
  // Serial.println(" Fahrenheit")

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

  File dataFile = SD.open("G5.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {

    int sensor = analogRead(A1);
    sensor = constrain(sensor, 350, 1023);
    // print out the value you read:
    //Serial.println(sensorValue);

    //map the value to a percentage
    soil = map(sensor, 350, 1023, 100, 0);

    //Print node ID here

    //clock start//
    now = RTC.now();

    dataFile.print(now.year(), DEC);
    dataFile.print('/');
    dataFile.print(now.month(), DEC);
    dataFile.print('/');
    dataFile.print(now.day(), DEC);
    dataFile.print(' ');
    dataFile.print(now.hour(), DEC);
    dataFile.print(':');
    dataFile.print(now.minute(), DEC);
    dataFile.print(':');
    dataFile.print(now.second(), DEC);
    dataFile.print(" , ");
    //clock end//

    //SENSOR DATA_SM_ST_AT_RH//
    dataFile.print("SM=")  ;
    dataFile.print(soil);
    dataFile.print(", ");
    dataFile.print("ST=");
    dataFile.print(celsius);
    dataFile.print(", ");

#if DEBUG
    Serial.print("S_Moisture=")  ;
    Serial.print(soil);
    Serial.print(",");
#endif
    int chk = dht.read(DHT22_PIN);

    dataFile.print("AT= ");
    AT = dht.readTemperature();
    dataFile.print(AT);
    dataFile.print(", ");
    dataFile.print("RH= ");
    RH = dht.readHumidity();
    dataFile.print(RH);
#if DEBUG
    Serial.print("Temperature = ");
    Serial.print(AT);
    Serial.print(",");
    Serial.print("RH= ");
    Serial.println(RH);
#endif

    /*EC start*/
    voltage = analogRead(A0) * (5.0 / 1023.0);
#if DEBUG
    Serial.print("EC=");
    Serial.println(voltage);
#endif
    dataFile.print(", ");
    dataFile.print("EC=");
    dataFile.println(voltage);
    //EC end//
    delay(100);
    dataFile.close();

#if PRODUCTION    

    if (count >= 7){ //14 minutes over, let us transmit
      	count = 0;

	if (connected == false)
	{
  	    digitalWrite(WIFI_ENABLE_PIN, HIGH);
	    delay(4000); /*Sleep For 4 Sec*/
	    connectWiFi();
	    delay(8000); //@ LOKESH CHANGE Wait for 8 seconds

	}

      	ValidatePost(soil, AT, RH, celsius, voltage);
	
	delay(4000); /*Sleep For 4 Sec*/

	disconnectWiFi(); /*viki: disconnecting from AP after post*/
        delay(4000); //Wait for 4 seconds
  	digitalWrite(WIFI_ENABLE_PIN, LOW);


#ifdef POWER_OPTIMIZATION
  	power_management_disable_ADC();
      	SLEEP_MULTIPLE_OF_8_SEC(15); /*Sleep for 15 * 8 seconds = 120 seconds*/
  	power_management_enable_ADC();
	delay(500);
#else
        delay(120000); //Wait for 2 minute
#endif

    }else{
    	count++;
#ifdef POWER_OPTIMIZATION
  	power_management_disable_ADC();
      	SLEEP_MULTIPLE_OF_8_SEC(15); /*Sleep for 15 * 8 seconds = 120 seconds*/
  	power_management_enable_ADC();
	delay(500);
#else
        delay(120000); //Wait for 2 minute
#endif
    }
#endif


#if DEVEL //For a quick testing
    ValidatePost(soil, AT, RH, celsius, voltage);
    delay(1000);
#endif   
  }
  // if the file isn't open, pop up an error:

  else {
#if DEBUG
    Serial.println("error opening data.txt");
#endif

  }
}


#ifdef POWER_OPTIMIZATION

/* Watch Dog Interrupt Callback 
 */
ISR(WDT_vect)
{
  /* Rememeber to add this function. 
   * This function is called after watch dog timer timeout.  
   * This is the callback/interrupt function called after waking up
   */
}

void power_management_enable_sleep_mode()
{
  //ENABLE SLEEP - this enables the sleep mode
  SMCR |= (1 << 2); //power down mode
  SMCR |= 1;//enable sleep

  /*Note!!: Sleep Instruction should be called after this*/
}

void power_management_disable_ADC()
{
  //Disable ADC - don't forget to flip back after waking up if using ADC in your application ADCSRA |= (1 << 7);
  ADCSRA &= ~(1 << 7);
}

void power_management_enable_ADC()
{
  ADCSRA |= (1 << 7);
}

/*Function: Makes ATMEL Chip wake up every 8 seconds*/
void power_management_init_watchdog_timer()
{
  //SETUP WATCHDOG TIMER
  WDTCSR = (24);//change enable and WDE - also resets
  WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit
  WDTCSR |= (1<<6);//enable interrupt mode
}

/* Put Atmel Chip in Sleep Mode for Approximately 8 Seconds until watchdog timer
 * wakes up and start resume the work.
 */
void power_management_goto_sleep()
{
  /*Disable Brown Out Detection Unit while Sleeping*/
  //BOD DISABLE - this must be called right before the __asm__ sleep instruction
  MCUCR |= (3 << 5); //set both BODS and BODSE at the same time
  MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time

  /* Put ATMEL Chip in Sleep Mode
   * InLine Assembler Code to go to Sleep
   */
  __asm__  __volatile__("sleep");//in line assembler to go to sleep 
}

#endif


