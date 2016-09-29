
/**
   Weather Station for Arduino
   @Author: José María Moreno Juez
   2016
   Components used:
   - Temp / Humidity DHT 11 sensor
   - Wifi module ESP8266
   - BMP180 Barometer / Temperature sensor
   - DS1302 Real Time Clock
   - One LCD screen 16x2 --> http://www2.joinville.udesc.br/~i9/2014/07/17/hands-on-uma-pequena-aula-lcd-para-arduino/

   This program sends data to a WeeWx server ( http://www.weewx.com/ ) running on a Raspberry pi on the same network
*/

#include <EEPROM.h>           // Library to deal with EEPROM memory
#include <Timer.h>            // Library for creating timers
#include <virtuabotixRTC.h>   // Library for DS1302 RTC component
#include <TimeLib.h>          // Library with time utilities
#include <Time.h>
#include <DHT.h>              // Library for dealing with DHT11 sensor
#include <SoftwareSerial.h>   // Library for dealing with ESP8266 WIFI module over serial communication
#include <Adafruit_BMP085.h>  // Library for the BMP180 Barometer sensor
#include <LiquidCrystal.h>    // Library for dealing with LCD screens
#include <ESP8266.h>          // Library to deal with module ESP8266 --> https://github.com/itead/ITEADLIB_Arduino_WeeESP8266

#define DEBUG           1           // Debug flag for showing messages to serial connection
#define BAUD_RATE       115200      // Serial port communication baud rate
#define DHTTYPE         DHT11       // DHT 11 (Humidity sensor)

// Digital pins used
#define LCD_RS_PIN      2           
#define LCD_EN_PIN      3           
#define LCD_D4_PIN      4           
#define LCD_D5_PIN      5           
#define LCD_D6_PIN      6           
#define LCD_D7_PIN      7           
#define DHT_PIN         8           // Pin used for DHT 11 sensor
#define ESPTX           9           // Transmission PIN for ESP8266
#define ESPRX           10          // Receive PIn for ESP8266
#define RTC_SCLK_PIN    11          // Pin used for RTC clock
#define RTC_IO_PIN      12          // I/O Pin for the RTC 
#define RTC_CE_PIN      13          // CE pin for the RTC module 

// Analog Pins used
#define LCD_BUTTONS_PIN 0           // Analog input where LCD buttons are attached
#define SDA_PIN         4           // IC2 data pin on Arduino Nano Analog Input
#define SCL_PIN         5           // IC2 clock pin on Arduino Nano Analog Input

                 
// Constants for the LCD buttons
#define btnRIGHT        0
#define btnUP           1
#define btnDOWN         2
#define btnLEFT         3
#define btnSELECT       4
#define btnNONE         5

#define LCD_COLUMNS     16
#define LCD_ROWS        2
#define LCD_CONTRAST    122

#ifdef DEBUG
#define SENSOR_READ_DELAY 10000           // When do we read sensor's data (in millisecs)? 10 seconds in debug mode
#else
#define SENSOR_READ_DELAY 60000           // When do we read sensor's data (in millisecs)? Default one minute
#endif
// XXX: In order to prolong EEPROM's life write to it as less as you can!!
#define WRITE_VARS_EEPROM_PERIOD 3600000  // When do we write variables to EEPROM (once an hour)

#define SSID_NAME       "MarensHouse"
#define WIFI_PASSWORD   "A952E4C9"
#define WEATHER_SERVER  "192.168.1.30"
#define WEATHER_PORT    (9999)


// Debug routines
#if DEBUG == 1
#define dprint(expression) Serial.print(F("# ")); Serial.print( #expression ); Serial.print( F(": ") ); Serial.println( expression )
#define dshow(expression) Serial.println( F(expression) )
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
const unsigned long CRC_STAMP = 2185581076L; // CRC of the 4 first bytes of the EEPROM
tmElements_t tm;              // Time struct
// Variable that stores the averages 
float avg_humidity = 0.0f;
float avg_temperature = 0.0f;
float avg_pressure = 0.0f;
float avg_altitude = 0.0f;
unsigned long measures_taken = 0;
Adafruit_BMP085 bmp;              // Variable to deal with BMP180 Barometer sensor
Timer t;                          // Instantiate the timer object
virtuabotixRTC myRTC(RTC_SCLK_PIN, 
                     RTC_IO_PIN,
                     RTC_CE_PIN); // Creation of the Real Time Clock Object
LiquidCrystal lcd(LCD_RS_PIN, 
                  LCD_EN_PIN, 
                  LCD_D4_PIN, 
                  LCD_D5_PIN, 
                  LCD_D6_PIN, 
                  LCD_D7_PIN);    // select the pins used on the LCD panel
DHT dht(DHT_PIN, DHTTYPE);        // Initialize DHT sensor for normal 16mhz Arduino
SoftwareSerial BT1(ESPRX, ESPTX); // Initialize serial communication for the WIFI module
ESP8266 wifi(BT1,BAUD_RATE);

/*************************** FUNCTION DECLARATION ********************************************/

int read_LCD_buttons(){               // read the buttons
    int adc_key_in = analogRead(LCD_BUTTONS_PIN);       // read the value from the sensor 

    // my buttons when read are centered at these values: 0, 144, 329, 504, 741
    // we add approx 50 to those values and check to see if we are close
    // We make this the 1st option for speed reasons since it will be the most likely result

    if (adc_key_in > 1000) return btnNONE; 

    // For V1.1 us this threshold
     if (adc_key_in < 50)   return btnRIGHT;  
     if (adc_key_in < 195)  return btnUP; 
     if (adc_key_in < 380)  return btnDOWN; 
     if (adc_key_in < 555)  return btnLEFT; 
     if (adc_key_in < 790)  return btnSELECT;  

    return btnNONE;                // when all others fail, return this.
}

// Function that stores time part of the Date
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
    myRTC.setDS1302Time(tm.Second, tm.Minute, tm.Hour, tm.Wday, tm.Day, tm.Month, tmYearToCalendar(tm.Year));
    setTime(tm.Hour, tm.Minute ,tm.Second , tm.Day,  tm.Month, tm.Year);
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

String getCurrentDateTime() {
  myRTC.updateTime();
  String retval = "Current Date / Time:";
  retval += myRTC.dayofmonth;
  retval += "/";
  retval += myRTC.month;
  retval += "/";
  retval += myRTC.year;
  retval += " ";
  retval += myRTC.hours;
  retval += ":";
  retval += myRTC.minutes;
  retval += ":";
  retval += myRTC.seconds;
  return retval;
}

float readTemperature() {
  return bmp.readTemperature();
}

long readPressure() {
  return bmp.readPressure();
}

float readAltitude() {
  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  return bmp.readAltitude(101500);
}

void resetVariables() {
  avg_humidity    = 0.0f;
  avg_temperature = 0.0f;
  avg_pressure    = 0;
  avg_altitude    = 0.0f;
  measures_taken  = 0;
}

void clearEEPROM() {
  dshow("Clearing the EEPROM to zeros");
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  dshow("DONE clearing the EEPROM to zeros");
}

void  readVarsFromEEPROM() {
  int address = 4;
  EEPROM.get(address, avg_humidity);
  if (isnan(avg_humidity)) {
    dshow("Failed to read avg_humidity from EEPROM");
  }
  address += sizeof(float);
  EEPROM.get(address, avg_temperature);
  if (isnan(avg_temperature)) {
    dshow("Failed to read avg_temperature from EEPROM");
  }
  address += sizeof(float);
  EEPROM.get(address, avg_pressure);
  if (isnan(avg_pressure)) {
    dshow("Failed to read avg_pressure from EEPROM");
  }
  address += sizeof(float);
  EEPROM.get(address, avg_altitude);
  if (isnan(avg_altitude)) {
    dshow("Failed to read avg_altitude from EEPROM");
  }
  address += sizeof(float);
  EEPROM.get(address, measures_taken);
  if (isnan(avg_altitude)) {
    dshow("Failed to read avg_altitude from EEPROM");
  }
}

void writeMeasureToEEPROM() {
  dshow("Writing variables to EEPROM");
  int address = 4;
  dprint(avg_humidity);
  EEPROM.put(address, avg_humidity);
  address += sizeof(float);
  dprint(avg_temperature);
  EEPROM.put(address, avg_temperature);
  address += sizeof(float);
  dprint(avg_pressure);
  EEPROM.put(address, avg_pressure);
  address += sizeof(long);
  dprint(avg_altitude);
  EEPROM.put(address, avg_altitude);
  address += sizeof(float);
  dprint(measures_taken);
  EEPROM.put(address, measures_taken);
}


// Function that checks if the EEPROM memory has been initialized by this program before.
// If not it clears it and initialized the first two bytes with a particular stamp.
void initEEPROM() {
  dshow("Initializing the EEPROM memory");
  int address = 0;
  unsigned long crc = eeprom_crc(0, 4);
  dprint(crc);
  dprint(CRC_STAMP);
  // Check if the EEPROM has been initialized by our program
  if (crc != CRC_STAMP) {
    dshow("CRC of the four first bytes is not correct");
    clearEEPROM();
    EEPROM.write(0, 232);
    EEPROM.write(1, 19);
    EEPROM.write(2, 158);
    EEPROM.write(3, 38);
    dprint(eeprom_crc(0, 4));
  } else {
    dshow("CRC is correct so read average values from memory into variables");
    readVarsFromEEPROM();
  }
  dshow("Done initializing the EEPROM memory");
}

// Function that calculates the CRC of the bytes stored in the EEPROM
unsigned long eeprom_crc(unsigned int from_address, unsigned int to_address) {

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

void readSensors()
{
  dshow("************ Reading sensors**********");
  humidity = dht.readHumidity();
  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity)) {
    dshow("Failed to read humidity from DHT sensor!");
    return;
  }
  temperature = readTemperature();
  if (isnan(temperature)) {
    dshow("Failed to read temperature from LM35 sensor!");
    return;
  }
  pressure = readPressure();
  if (isnan(pressure)) {
    dshow("Failed to read pressure from barometric sensor!");
    return;
  }
  altitude = readAltitude();
   if (isnan(altitude)) {
    dshow("Failed to read altitude from barometric sensor!");
    return;
  }
  
  dprint(humidity);
  dprint(temperature);
  dprint(pressure);
  dprint(altitude);

  if (measures_taken == 0) {
    avg_humidity    = humidity;
    avg_temperature = temperature;
    avg_pressure    = pressure;
    avg_altitude    = altitude;
  } else { // How to calculate mean: http://math.stackexchange.com/questions/198469/how-the-sample-mean-changes-when-you-add-a-new-observation 
      avg_humidity =    (measures_taken * avg_humidity) / (measures_taken + 1)  + humidity / (measures_taken + 1);
      avg_temperature = (measures_taken * avg_temperature) / (measures_taken + 1)  + temperature / (measures_taken + 1);
      avg_pressure =    (measures_taken * avg_pressure) / (measures_taken + 1)  + pressure / (measures_taken + 1);
      avg_altitude =    (measures_taken * avg_altitude) / (measures_taken + 1)  + altitude / (measures_taken + 1);
      dshow("New average values are:");
      dprint(avg_humidity);
      dprint(avg_temperature); 
      dprint(avg_pressure);
      dprint(avg_altitude);
      dshow("**********************");
  }
  measures_taken++; // Increase the number of measures taken
  dshow("************ Succesfully read sensors **********");
  sendDataWIFI(); // Finally send the data over WIFI to the server where WeeWx is running
  
}

// Reset the esp8266 module
bool setupWIFI() { 
  dshow("Setting up WIFI module over serial");
  // Set operation mode for module
  if (wifi.setOprToStationSoftAP()) {
    dshow("to station + softap ok");
  } else {
    dshow("to station + softap err");
  }
  // Connect to the access point
  if (wifi.joinAP(SSID_NAME, WIFI_PASSWORD)) {
    dshow("Join AP success");
    dprint(wifi.getLocalIP());       
  } else {
     dshow("Join AP failure");
     return false;
  }
  // Disable multiple connections
  if (wifi.disableMUX()) {
    dshow("single ok");
  } else {
    dshow("single err");
  }
  dshow("Wifi setup end");
  return true;
}

// Function that sends data to the WeeWx server over TCP port 9999
bool sendDataWIFI() {
  bool isConnected = false;
  bool retVal = false;
  dshow("Sending data over WIFI");
  if (wifi.createTCP(WEATHER_SERVER, WEATHER_PORT)) {
    dshow("Connected to the weather host");
    isConnected = true;
  } else {
    dshow("Can't connect to the server");
    return false;
  }
  /* Compose data to be sent to the WeeWx server
   *  It should have the form of
   *  key=value
   *  key=value...
  */
  char temp[10];
  // WeeWx FileParser expect measures using imperial system
  String data = "Date = " + String(now()) + "\n"; 
  dtostrf(celsius2fahrenheit(temperature),1,2,temp);
  data += "outTemp = " +  String(temp) + "\n";
  dtostrf(humidity,1,2,temp);
  data += "outHumidity = " +   String(temp) + "\n";
  float pressureTemp = pressure *0.0002953337;
  dtostrf(pressureTemp,1,2,temp);
  data += "pressure = " +   String(temp) + "\n";
  dtostrf(altitude,1,2,temp);
  data += "altimeter = " +   String(temp) + "\n";
  if (wifi.send((const uint8_t*)data.c_str(), data.length())) {
    dshow("Data successfully sent to the server");
  } else {
    dshow("Error sending data to the server");
  }
  // close the connection
  dshow("Attempting to disconnect from server");
  if (isConnected) {
    if (wifi.releaseTCP()) {
      dshow("release tcp ok");
      retVal = true;
    } else {
      dshow("release tcp err");
    }
  }
  return retVal;
}

float celsius2fahrenheit (float celsius){
  return (1.8 * celsius) + 32;
}

/*************************** END OF FUNCTION DECLARATION ********************************************/

void setup() { // put your setup code here, to run once:
 // Init serial port
  Serial.begin(BAUD_RATE);
  while (!Serial) ; // wait for Arduino Serial Monitor
  dshow("Setting up weather station v 1.0!");
  initEEPROM(); // Initialize the EEPROM
  setupWIFI();
  // connectWIFI(); // FIXME: 
  setRTCFromCompiler(); // Initialize RTC clock from compiler time
  lcd.begin(LCD_COLUMNS,LCD_ROWS); // Set up LCD display
  if (!bmp.begin()) { // TODO: Could pass BMP085_ULTRALOWPOWER to save battery
    dshow("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  dht.begin(); // Initialize DHT 11 sensor
  // Attach events to the timer
  t.every(SENSOR_READ_DELAY, readSensors);
  t.every(WRITE_VARS_EEPROM_PERIOD, writeMeasureToEEPROM);
}

void loop() {
  myRTC.updateTime();
  // analogWrite (10, fadeValue);    // change the contrast
  lcd.setCursor(9,1);             // move cursor to second line "1" and 9 spaces over
  lcd.print(millis()/1000);       // display seconds elapsed since power-up
  lcd.setCursor(0,1);             // move to the begining of the second line
  int lcd_key = read_LCD_buttons();   // read the buttons
  switch (lcd_key){               // depending on which button was pushed, we perform an action
       case btnRIGHT:{             //  push button "RIGHT" and show the word on the screen
            lcd.print(F("RIGHT "));
            break;
       }
       case btnLEFT:{
             lcd.print(F("LEFT   ")); //  push button "LEFT" and show the word on the screen
             break;
       }    
       case btnUP:{
             lcd.print(F("UP    "));  //  push button "UP" and show the word on the screen
             break;
       }
       case btnDOWN:{
             lcd.print(F("DOWN    "));  //  push button "UP" and show the word on the screen
             break;
       }
       case btnSELECT:{
             lcd.print(F("SELECT"));  //  push button "SELECT" and show the word on the screen
             break;
       }
       case btnNONE:{
             lcd.print(F("NONE  "));  //  No action  will show "None" on the screen
             break;
       }
  }
  t.update();
}
