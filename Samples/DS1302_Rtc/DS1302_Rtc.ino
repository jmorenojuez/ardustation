#include <virtuabotixRTC.h> // Library
#include <TimeLib.h>

#define RTC_SCLK_PIN  6
#define RTC_IO_PIN  7
#define RTC_CE_PIN  8
// Creation of the Real Time Clock Object
//SCLK -> 6, I/O -> 7, CE -> 8
virtuabotixRTC myRTC(RTC_SCLK_PIN, RTC_IO_PIN, RTC_CE_PIN);

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

tmElements_t tm;
 
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
  Serial.print(Year);
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
     Serial.print("DS1302 configured Time=");
     Serial.print(__TIME__);
     Serial.print(", Date=");
     Serial.println(__DATE__);
  } else {
    Serial.print("Could not parse info from the compiler, Time=\"");
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
    Serial.println("\"");
  }

  
}


void setup() {
  // Setup serial port
  Serial.begin(9600);
  while (!Serial) ; // wait for Arduino Serial Monitor
  delay(200);
  setRTCFromCompiler();
}

void loop() {
  // This allows for the update of variables for time or accessing the individual elements.
  myRTC.updateTime();
  // Start printing elements as individuals
  Serial.print("Current Date / Time: ");
  Serial.print(myRTC.dayofmonth);
  Serial.print("/");
  Serial.print(myRTC.month);
  Serial.print("/");
  Serial.print(myRTC.year);
  Serial.print(" ");
  Serial.print(myRTC.hours);
  Serial.print(":");
  Serial.print(myRTC.minutes);
  Serial.print(":");
  Serial.println(myRTC.seconds);
  
  // Delay so the program doesn't print non-stop
  delay( 5000);
}
