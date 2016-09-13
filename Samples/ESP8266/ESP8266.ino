#include <SoftwareSerial.h>
SoftwareSerial BT1(3, 2); // RX | TX

//#define BAUD_RATE 9600
#define BAUD_RATE 115200

String W =" ";
char w ;

void setup(){  
    Serial.begin(BAUD_RATE);
    while (!Serial) ; // wait for Arduino Serial Monitor
    BT1.begin(BAUD_RATE);
    pinMode(13, OUTPUT);
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
