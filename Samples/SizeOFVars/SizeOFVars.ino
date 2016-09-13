#include <stdint.h>

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("sizeof(byte)=");
  Serial.println(sizeof(byte));
  Serial.println();
  
  Serial.print("sizeof(char)=");
  Serial.println(sizeof(char));
  Serial.println();
  
  Serial.print("sizeof(short)=");
  Serial.println(sizeof(short));
  Serial.println();
  
  Serial.print("sizeof(int)=");
  Serial.println(sizeof(int));
  Serial.println();
  
  Serial.print("sizeof(long)=");
  Serial.println(sizeof(long));
  Serial.println();
  
  Serial.print("sizeof(long long)=");
  Serial.println(sizeof(long long));
  Serial.println();
  
  Serial.print("sizeof(bool)=");
  Serial.println(sizeof(bool));
  Serial.println();
  
  Serial.print("sizeof(boolean)=");
  Serial.println(sizeof(boolean));
  Serial.println();
  
  Serial.print("sizeof(float)=");
  Serial.println(sizeof(float));
  Serial.println();
  
  Serial.print("sizeof(double)=");
  Serial.println(sizeof(double));
  Serial.println();
  
  Serial.print("sizeof(int8_t)=");
  Serial.println(sizeof(int8_t));
  Serial.println();
  
  Serial.print("sizeof(int16_t)=");
  Serial.println(sizeof(int16_t));
  Serial.println();
  
  Serial.print("sizeof(int32_t)=");
  Serial.println(sizeof(int32_t));
  Serial.println();
  
  Serial.print("sizeof(int64_t)=");
  Serial.println(sizeof(int64_t));
  Serial.println();
  
  Serial.print("sizeof(uint8_t)=");
  Serial.println(sizeof(uint8_t));
  Serial.println();
  
  Serial.print("sizeof(uint16_t)=");
  Serial.println(sizeof(uint16_t));
  Serial.println();
  
  Serial.print("sizeof(uint32_t)=");
  Serial.println(sizeof(uint32_t));
  Serial.println();
  
  Serial.print("sizeof(uint64_t)=");
  Serial.println(sizeof(uint64_t));
  Serial.println();
  
  Serial.print("sizeof(char*)=");
  Serial.println(sizeof(char*));
  Serial.println();
  
  Serial.print("sizeof(int*)=");
  Serial.println(sizeof(int*));
  Serial.println();
  
  Serial.print("sizeof(long*)=");
  Serial.println(sizeof(long*));
  Serial.println();
  
  Serial.print("sizeof(float*)=");
  Serial.println(sizeof(float*));
  Serial.println();
  
  Serial.print("sizeof(double*)=");
  Serial.println(sizeof(double*));
  Serial.println();
  
  Serial.print("sizeof(void*)=");
  Serial.println(sizeof(void*));
  Serial.println();
  
  while (1)
    delay(10000);
}
