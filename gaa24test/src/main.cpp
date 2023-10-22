#include <Arduino.h>
#include <Servo.h>
//#define SD_CS 16 
#define LSM_CS 9
#define LSM_INT_1 10
#define LSM_INT_2 11
#define LPS_CS 15
#define LPS_INT 12
#define SW1 26
#define SW2 22
//using adafruit's libraries

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Bounce2.h>
// put function declarations here:
const int _MISO = 20;
const int _MOSI = 19;
const int _CS = 16;
const int _SCK = 18;

#define LPS_DAT_RATE LPS22_RATE_50_HZ
#define IMU_DAT_RATE LSM6DS_RATE_52_HZ

#include <SPI.h>
//#include <SD.h>
Adafruit_LSM6DSOX sox;
Adafruit_LPS22 lps;
Servo srv;
Bounce sw1, sw2;
void imuDatRdy();
void baroDatRdy();


void setup() {
  attachInterrupt(LSM_INT_1,&imuDatRdy,HIGH);
  attachInterrupt(LPS_INT,&baroDatRdy,HIGH);
  

  Serial.begin(115200);

  Serial.println("hello!!!!");
  Serial.println(srv.attach(4));
  pinMode(25, OUTPUT);
  digitalWrite(25,HIGH);
  
  sw1.attach(SW1,INPUT_PULLUP);
  sw1.interval(5);
  sw2.attach(SW2,INPUT_PULLUP);
  sw2.interval(5);

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
sox.configInt1(0,1,1);

  if (!lps.begin_SPI(LPS_CS)) {
    while(1){
      Serial.println("nooooooo LPS didnt work");
      delay(100);
    }
  }


  sox.setAccelDataRate(IMU_DAT_RATE);
  sox.setGyroDataRate(IMU_DAT_RATE);

  lps.setDataRate(LPS_DAT_RATE);


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
  sw1.update();
  sw2.update();

  srv.write(90);
  digitalWrite(25,HIGH);
    delay(100);                      // wait for a second
  digitalWrite(25, LOW);   // turn the LED off by making the voltage LOW
         
  srv.write(0);
   delay(100);  
  Serial.println("SW1 state:");
  Serial.println(sw1.read());
  Serial.println("SW2 state:");
  Serial.println(sw2.read());
}






void imuDatRdy(){
 //display imu dats
  Serial.println("IMU data");
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
}

void baroDatRdy(){
  Serial.println("LPS data");
  sensors_event_t temper;
  sensors_event_t pressure;
  lps.getEvent(&pressure, &temper);// get pressure
  Serial.print("Temperature: ");Serial.print(temper.temperature);Serial.println(" degrees C");
  Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa");
  Serial.println("");
}