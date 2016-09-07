


/**
 * Weather Station fro Arduino
 * Components used:
 * - One green LED conected to PIN 12 -- Indicates succesfull operation
 * - One red LED conected to PIN 11 -- Indicates failure operation 
 * - Temp / Humidity DHT 11 sensor
 * - Wifi module ESP8266
 */
#include <Timer.h> // Library for creating timers
#include "DHT.h" // Library for dealing with DHT11 sensor
#include <SoftwareSerial.h> // Library for dealing with serial comunications
#include <Wire.h>
#include <Adafruit_BMP085.h> // Library for the BMP180 Barometer sensor

#define DHTPIN 2     // what pin we're connected to
#define SDAPIN  4    // IC2 data pin on Arduino Nano Analog
#define SCLPIN  5    // IC2 clock pin on Arduino Nano Analog

#define ESPRX  6     // what pin we're connected to
#define ESPTX  5     // what pin we're connected to

#define DHTTYPE DHT11   // DHT 11 (Humidity sensor)

#define LED_RED 3     // what pin we're connected to
#define LED_GREEN 4     // what pin we're connected to
const int LM35Pin = A0; // Analog pin for LM35 sensor

#define SENSOR_READ_DELAY 1000 // When do we read sensor's data (in millisecs) ?

// Declare global variables
float humidity = 0.0f; // Read from DHT11 sensor
float temperature = 0.0f; // Read from LM35 sensor
long pressure = 0; // TODO: 
float altitude; 
 
Adafruit_BMP085 bmp;
Timer t; //instantiate the timer object

// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);

// Initialize serial communication
SoftwareSerial BT1(ESPRX, ESPTX); // RX | TX

float readTemperatureFromLM35() {
   int value = analogRead(LM35Pin);
   float millivolts = (value / 1023.0) * 5000;
   float celsius = millivolts / 10;
   return celsius; 
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

void setup() {
  // initialize digital pins for LEDS as an output.
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  // put your setup code here, to run once:
  turnOffLeds();
  Serial.begin(9600); 
  BT1.begin(9600);
  Serial.println("Starting weather station v 1.0!");
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  dht.begin();
  t.every(SENSOR_READ_DELAY, readSensors);
  
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

void readSensors() 
{  
  turnOffLeds();
  // put your main code here, to run repeatedly:
  humidity = dht.readHumidity();
  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity)) {
    Serial.println("Failed to read humidity from DHT sensor!");
  }
  temperature = readTemperatureFromLM35();
  if (isnan(temperature)) {
    Serial.println("Failed to read temperature from LM35 sensor!");
  }
  pressure = readPressure();
  altitude = readAltitude();
  turnOnGreenLed();
  Serial.print("Humidity from DHT11: "); 
  Serial.print(humidity);
  Serial.print("%\t");
  Serial.print("Temperature from LM35: "); 
  Serial.print(temperature);
  Serial.print(" C\n");
  Serial.print("Pressure from BMP180: "); 
  Serial.print(pressure);
  Serial.print(" Pa\n");
  Serial.print("Altitude from BMP180: "); 
  Serial.print(altitude);
  Serial.print(" meters\n");
}

