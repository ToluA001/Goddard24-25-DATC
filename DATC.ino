#include <Adafruit_HDC302x.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include "RTClib.h"
#include <SD.h>
#include <SoftwareSerial.h>
#include "math.h"
#include "stdint.h"
//Pressure transducer setup
#define ANALOG_READ_RESOLUTION_BITS 14
#define BAUD_RATE 9600
#define SENSOR_DELAY_MS 250

#define NOMINAL_TEMP_C 25.0     // Calibration temp
#define TEMP_COEFF_ZERO 0.0005  // 0.05% FS/°C
#define TEMP_COEFF_SENS 0.0005  // 0.05% FS/°C

float TEMP = 20.5;  // Room temperature in Celsius (e.g., 70°F ≈ 21.1°C)
typedef struct {
  float V_LOW;
  float V_HI;
  float PRESSURE_RANGE;
  int AI_PIN;
} PRESSURE_TRANSDUCER;

PRESSURE_TRANSDUCER pt1 = {
  0.5,   // V_LOW
  4.5,   // V_HI
  1600,  // PRESSURE_RANGE in PSI
  A0     // AI_PIN
};

typedef struct {
  uint16_t raw;
  float voltage;
  float pressure_psi;
  float drift_psi;
} DATA;

DATA readData(PRESSURE_TRANSDUCER pt) {
  uint16_t raw = analogRead(pt.AI_PIN);
  float voltage = raw * (5.0 / pow(2, ANALOG_READ_RESOLUTION_BITS));

  float pressure_psi = pt.PRESSURE_RANGE * ((voltage - pt.V_LOW) / (pt.V_HI - pt.V_LOW));
  if (pressure_psi < 0) pressure_psi = 0;

  float temp_delta = TEMP - NOMINAL_TEMP_C;
  float drift_psi = pt.PRESSURE_RANGE * (fabs(temp_delta) * (TEMP_COEFF_ZERO + TEMP_COEFF_SENS));

  return {
    raw,
    voltage,
    pressure_psi,
    drift_psi
  };
}

void printData(DATA data) {
  Serial.print("Raw: ");
  Serial.print(data.raw);
  Serial.print(" | Voltage: ");
  Serial.print(data.voltage, 3);
  Serial.print(" V | Pressure: ");
  Serial.print(data.pressure_psi, 2);
  Serial.print(" PSI ±");
  Serial.print(data.drift_psi, 2);
  Serial.println(" PSI (temp error)");
}
//end of transducer setup

RTC_DS3231 rtc;

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
SoftwareSerial xbee(0, 1); // RX, TX

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

Adafruit_ICM20649 icm;
Adafruit_HDC302x hdc = Adafruit_HDC302x();

const int chipSelect = 10;
File myFile;

void setup() 
{
  Serial.begin(115200);
  xbee.begin(115200);

  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif
  if (!hdc.begin(0x44, &Wire) || !icm.begin_I2C(0x69, &Wire) || !mpr.begin(0x18, &Wire) || !rtc.begin())
  {
    Serial.println("Could not find one or more Sensors");
    while(1); 
  }

  if (!SD.begin(chipSelect)) 
  {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  }


  if (rtc.lostPower()) 
  {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  switch (icm.getAccelRange()) 
  {
    case ICM20649_ACCEL_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case ICM20649_ACCEL_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case ICM20649_ACCEL_RANGE_16_G:
      Serial.println("+-16G");
      break;
    case ICM20649_ACCEL_RANGE_30_G:
      Serial.println("+-30G");
      break;
  }

  //icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  delay(1000);
}

void loop() 
{
  myFile = SD.open("myfile.csv", FILE_WRITE);




  DateTime now = rtc.now();
  
  char line[200];
  char lineT[100];

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp);

  double temper = 0.0;
  double RH = 0.0;

  hdc.readTemperatureHumidityOnDemand(temper, RH, TRIGGERMODE_LP0);
  

  //transducer code
  TEMP==temper;
  analogReadResolution(ANALOG_READ_RESOLUTION_BITS);
  delay(10);

  DATA data_1 = readData(pt1);

  Serial.print("Sensor 1 -> ");
  printData(data_1);

  Serial.println("------------------");
  delay(SENSOR_DELAY_MS);

  //end of transducer code

  /*Serial.print("Temperature: ");
  Serial.print(temper);
  Serial.println(" °C");*/

  /*Serial.print("Humidity: ");
  Serial.print(RH);
  Serial.println(" %");*/
  

  //mprls
  float pressure_hPa = mpr.readPressure();
  //Serial.print("Pressure (hPa): "); Serial.println(pressure_hPa);
  //Serial.print("Pressure (PSI): "); Serial.println(pressure_hPa / 68.947572932);

  /*Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();*/

  sprintf(line, "Hour: %i, Min: %i, Sec: %i, Pressure: %f, AX: %f, AY: %f, AZ: %f, GX: %f, GY: %f, GY: %f, Humidity: %f, Temp(c): %f, Oxpressure(psi): %f, Transvoltage(V): %f", now.hour(), now.minute(), now.second(), pressure_hPa, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, RH, temper,data_1.pressure_psi,data_1.voltage);
  Serial.println(line);

  while (xbee.available()) 
  {
    char c = xbee.read();
    Serial.print("Received: ");
    Serial.println(c);
  }

  if(myFile)
  {
    Serial.println("Line saved");
    myFile.println(line);
    myFile.close();
  }
  else 
  {
    Serial.println("error opening test.csv");
  }
  
  xbee.println(line);
  
  delay(1000);
}
