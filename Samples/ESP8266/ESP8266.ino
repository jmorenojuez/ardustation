#include <SoftwareSerial.h>

#define ESPTX           9           // Transmission PIN for ESP8266
#define ESPRX           10          // Receive PIn for ESP8266

SoftwareSerial BT1(ESPRX, ESPTX); // RX | TX

//#define BAUD_RATE 9600
#define BAUD_RATE 115200

String W =" ";
char w ;

void setup(){  
    Serial.begin(BAUD_RATE);
    while (!Serial) ; // wait for Arduino Serial Monitor
    BT1.begin(BAUD_RATE);
    pinMode(13, OUTPUT);
    Serial.println("Finished setup");
}

void loop()
{ 
  String B= "." ;
     if (BT1.available())
         { char c = BT1.read() ;
           Serial.print(c);
         }
     if (Serial.available())
         {  char c = Serial.read();
            BT1.print(c);
         }                    // Limpiamos las variables
}
