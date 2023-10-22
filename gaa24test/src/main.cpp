#include <Arduino.h>
#include <Servo.h>
//#define SD_CS 16 
#define LSM_CS 15
#define LPS_CS 21
//using adafruit's libraries

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>

// put function declarations here:
const int _MISO = 20;
const int _MOSI = 19;
const int _CS = 16;
const int _SCK = 18;

#include <SPI.h>
//#include <SD.h>
Adafruit_LSM6DSOX sox;
Adafruit_LPS22 lps;
Servo srv;
void setup() {
  Serial.begin(115200);

  Serial.println("hello!!!!");
  Serial.println(srv.attach(4));
  pinMode(25, OUTPUT);
  digitalWrite(25,HIGH);
 
  SPI.setRX(_MISO);
  Serial.println(":/");
  SPI.setTX(_MOSI);
  Serial.println(":-)");
  SPI.setSCK(_SCK);
Serial.println(":)");
  if (!sox.begin_SPI(LSM_CS)) {
    while(1){
      Serial.println("nooooooo sox didnt work");
      delay(100);
    }
  }
  if (!lps.begin_SPI(LPS_CS)) {
    while(1){
      Serial.println("nooooooo LPS didnt work");
      delay(100);
    }
  }


  sox.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);

  lps.setDataRate(LPS22_RATE_75_HZ);
  
  // see if the card is present and can be initialized:
  /**
  if (!SD.begin(_CS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while(1){
      delay(15);
      Serial.println("Card failed, or not present");
    }
  }
  Serial.println("card initialized.");*/
  
}

void loop() {
  srv.write(90);
  digitalWrite(25,HIGH);
    delay(15);                      // wait for a second
  digitalWrite(25, LOW);   // turn the LED off by making the voltage LOW
         
  srv.write(0);
   delay(15);  
  Serial.println("IMU data");

  //display imu dats
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println("");

  Serial.println("LPS data");
  sensors_event_t temper;
  sensors_event_t pressure;
  lps.getEvent(&pressure, &temper);// get pressure
  Serial.print("Temperature: ");Serial.print(temper.temperature);Serial.println(" degrees C");
  Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa");
  Serial.println("");

}









