/**
 * Weather Station for Arduino
 * @Author: José María Moreno Juez
 * 2016
 * Components used:
 * - One green LED conected to PIN 12 -- Indicates succesfull operation
 * - One red LED conected to PIN 11 -- Indicates failure operation 
 * - Temp / Humidity DHT 11 sensor
 * - Wifi module ESP8266
 * - BMP180 Barometer / Temperature sensor
 * - DS1302 Real Time Clock
 * - One LCD 16x2
 */

#include <EEPROM.h>           // Library to deal with EEPROM memory
#include <Timer.h>            // Library for creating timers
#include <virtuabotixRTC.h>   // Library for DS1302 RTC component
#include <TimeLib.h>          // Library with time utilities
#include <DHT.h>              // Library for dealing with DHT11 sensor
#include <SoftwareSerial.h>   // Library for dealing with serial comunications
#include <Wire.h>             // Library for dealing with LCD screens
#include <Adafruit_BMP085.h>  // Library for the BMP180 Barometer sensor

#define DEBUG 1
#define CRC_STAMP 2185581076  // CRC of the 4 first bytes of the EEPROM
#define BAUD_RATE 9600        // Serial port communication baud rate
#define DHTTYPE DHT11         // DHT 11 (Humidity sensor)
#define DHT_PIN 2             // Pin used for DHT 11 sensor
#define LED_RED 3             // Pin use for red LED
#define LED_GREEN 4           // Pin use for green LED
#define SDA_PIN  4            // IC2 data pin on Arduino Nano Analog
#define SCL_PIN  5            // IC2 clock pin on Arduino Nano Analog
#define ESPTX  5              // what pin we're connected to
#define ESPRX  6              // what pin we're connected to
#define RTC_SCLK_PIN  7       // Pin used for RTC clock
#define RTC_IO_PIN  8         // I/O Pin fro the RTC 
#define RTC_CE_PIN  9         // CE pin for the RTC module 

#define SENSOR_READ_DELAY 1000 // When do we read sensor's data (in millisecs) ?
// FIXME: In order to prolong to EEPROM life write to it as less as you can
#define WRITE_VARS_EEPROM_PERIOD 1000 // When do we read sensor's data (in millisecs) 

// Debug routine
#if DEBUG == 1
#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression )
#define dshow(expression) Serial.println( expression )
#else
#define dprint(expression)
#define dshow(expression)
#endif

// Array of string with month names
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

// Declare global variables
float humidity = 0.0f;        // Read from DHT11 sensor
float temperature = 0.0f;     // Read from LM35 sensor
long pressure = 0;            // Stores current air pressure 
float altitude = 0.0f;        // Stores current altitude

tmElements_t tm;              // Time struct

float avg_humidity = 0.0f;
float avg_temperature = 0.0f;
long avg_pressure = 0;
float avg_altitude = 0.0f;
unsigned long measures_taken = 0;
 
Adafruit_BMP085 bmp;          // Variable to deal with BMP180 Barometer sensor
Timer t;                      // Instantiate the timer object
virtuabotixRTC myRTC(RTC_SCLK_PIN, RTC_IO_PIN, RTC_CE_PIN); // Creation of the Real Time Clock Object
DHT dht(DHT_PIN, DHTTYPE);      // Initialize DHT sensor for normal 16mhz Arduino
SoftwareSerial BT1(ESPRX, ESPTX); // // Initialize serial communication

/*************************** FUNCTION DECLARATION ********************************************/
// Function that stores 
bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}

void setRTCFromCompiler() {
  // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__)) {
     // Set the current date, and time in the following format:
     // seconds, minutes, hours, day of the week, day of the month, month, year
     myRTC.setDS1302Time(tm.Second, tm.Minute, tm.Hour, tm.Wday,tm.Day,tm.Month, tmYearToCalendar(tm.Year));
     dshow("DS1302 configured Time=");
     dshow(__TIME__);
     dshow(", Date=");
     dshow(__DATE__);
  } else {
    dshow("Could not parse info from the compiler, Time=\"");
    dshow(__TIME__);
    dshow("\", Date=\"");
    dshow(__DATE__);
  }
}

const char *getRTCDateTime(virtuabotixRTC rtc){
  String retval = "Current Date / Time:";
  retval += rtc.dayofmonth;
  retval += "/";
  retval += rtc.month;
  retval += "/";
  retval += rtc.year;
  retval += " ";
  retval += rtc.hours;
  retval += ":";
  retval += rtc.minutes;
  retval += ":";
  retval += rtc.seconds;
  return retval.c_str();   
}

float readTemperature() {
  return bmp.readTemperature();  
}

long readPressure(){
  return bmp.readPressure();
}

float readAltitude(){
  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  return bmp.readAltitude(101500);
}

void turnOffLeds() {
    digitalWrite(LED_RED,LOW);
    digitalWrite(LED_GREEN,LOW);
}

void turnOnGreenLed() {
  digitalWrite(LED_GREEN,HIGH);
}

void turnOnRedLed() {
  digitalWrite(LED_RED,HIGH);
}

void clearEEPROM() {
  dshow("Clearing the EEPROM to zeros");
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }  
}
void  readVarsFromEEPROM(){
  int address = 4;
  EEPROM.get(address,avg_humidity);
  if (isnan(avg_humidity)) {
    dshow("Failed to read avg_humidity from EEPROM");
  }
  address += sizeof(float);
  EEPROM.get(address,avg_temperature);
  if (isnan(avg_temperature)) {
    dshow("Failed to read avg_temperature from EEPROM");
  }
  address += sizeof(float);
  EEPROM.get(address,avg_pressure);
  if (isnan(avg_pressure)) {
    dshow("Failed to read avg_pressure from EEPROM");
  }
  address += sizeof(long);
  EEPROM.get(address,avg_altitude);
  if (isnan(avg_altitude)) {
    dshow("Failed to read avg_altitude from EEPROM");
  }
  address += sizeof(float);
  EEPROM.get(address,measures_taken);
  if (isnan(avg_altitude)) {
    dshow("Failed to read avg_altitude from EEPROM");
  }
}


void writeMeasureToEEPROM(){
  dshow("Writing variables to EEPROM");
  int address = 4;
  EEPROM.put(address,avg_humidity);
  address += sizeof(float);
  EEPROM.put(address,avg_temperature);
  address += sizeof(float);
  EEPROM.put(address,avg_pressure);
  address += sizeof(long);
  EEPROM.put(address,avg_altitude);
  address += sizeof(float);
  EEPROM.put(address,measures_taken);
}


// Function that checks if the EEPROM memory has been initilialized by this program before. 
// If not it clears it and init the first two bytes with a particular stamp.
void initEEPROM() {
  dshow("Initilizing the EEPROM memory");
  int address = 0;
  long crc = eeprom_crc(0,3);
  // Check if the EEPROM has been initialzed by our program
  if (crc != CRC_STAMP) {
    dshow("CRC of the four first bytes is not correct");
    clearEEPROM();
    EEPROM.write(0,232);
    EEPROM.write(1,19);
    EEPROM.write(2,158);
    EEPROM.write(3,38);
    dprint(eeprom_crc(0,4));
  } else {
    dshow("CRC is correct so read average values from meory int ovariables");
    readVarsFromEEPROM();
  }
}

// Funciton that calculates the CRC of the bytes stored in the EEPROM
unsigned long eeprom_crc(unsigned int from_address,unsigned int to_address) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = from_address ; index < to_address  ; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

/*************************** END OF FUNCTION DECLARATION ********************************************/

void setup() { // put your setup code here, to run once:
  dshow("Setting up weather station v 1.0!");
  // Init serial port
  Serial.begin(BAUD_RATE); 
  while (!Serial) ; // wait for Arduino Serial Monitor
  initEEPROM();
  // initialize digital pins for LEDS as an output.
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  turnOffLeds();
  
  setRTCFromCompiler(); // Initialize RTC clock form compiler time
  BT1.begin(BAUD_RATE); // Initilize WIFI module via serial communication
  if (!bmp.begin()) { // TODO: Could pass BMP085_ULTRALOWPOWER to save battery
    dshow("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  dht.begin(); // Initialize DHT 11 sensor
  // Attacht events to the timer
  t.every(SENSOR_READ_DELAY, readSensors);
  t.every(WRITE_VARS_EEPROM_PERIOD, writeMeasureToEEPROM);
  
}

void loop() 
{
  t.update();
  if (BT1.available()){ 
    char c = BT1.read() ;
    Serial.print(c);
  }
  if (Serial.available()){  
    char c = Serial.read();
    BT1.print(c);
  }
}

void readSensors(){  
  turnOffLeds();
  humidity = dht.readHumidity();
  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity)) {
    dshow("Failed to read humidity from DHT sensor!");
  }
  temperature = readTemperature();
  if (isnan(temperature)) {
    dshow("Failed to read temperature from LM35 sensor!");
  }
  pressure = readPressure();
  altitude = readAltitude();
  turnOnGreenLed();
  dshow("Humidity from DHT11: "); 
  dshow(humidity);
  dshow("Temperature from LM35: "); 
  dshow(temperature);
  dshow(" C\n");
  dshow("Pressure from BMP180: "); 
  dshow(pressure);
  dshow(" Pa");
  dshow("Altitude from BMP180: "); 
  dshow(altitude);
  dshow(" meters");
  dshow(getRTCDateTime(myRTC));
}

