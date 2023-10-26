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
#define ROLLING_AVG_LEN 5
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Bounce2.h>
#include <numeric>
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
int imuInts;
int baroInts;

double Vx,Vy,Vz;
long long timebefore;
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  sensors_event_t temper;
  sensors_event_t pressure;



  class roll{//tested (it works)
  public:
    float acclRaw[3][ROLLING_AVG_LEN];
    float gyroRaw[3][ROLLING_AVG_LEN];
    float altitudeRaw[ROLLING_AVG_LEN];
   
   void clipVariables(float max,float ceiling,float min, float floor,int arrayToUse){ //case 1 = accl, 2= gyro, 3= altitude
    switch(arrayToUse){
      case(1): //accel data
      for (int j=0; j<3;j++){
        for (int i=0; i<ROLLING_AVG_LEN;i++){
          if (acclRaw[j][i]>=max){
            acclRaw[j][i]= ceiling;
          } else if (acclRaw[j][i]<=min){
            acclRaw[j][i]=floor;
          }
        }
      }
      
      break;

      case(2): //gyro data
      for (int j=0; j<3;j++){
        for (int i=0; i<ROLLING_AVG_LEN;i++){
          if (gyroRaw[j][i]>=max){
            gyroRaw[j][i]= ceiling;
          } else if (gyroRaw[j][i]<=min){
            gyroRaw[j][i]=floor;
          }
        }
      }
      break;

      case(3): //alt data

        for (int i=0; i<ROLLING_AVG_LEN;i++){
          if (altitudeRaw[i]>=max){
            altitudeRaw[i]= ceiling;
          } else if (altitudeRaw[i]<=min){
            altitudeRaw[i]=floor;
          }
        }
      
      break;
      
      default:
      break;

    }
   }

    void shiftArray(float newData, float *array, int size){
      for (int i=0; i<size-1; i++){ //downshift all values
         array[i]=array[i+1];
      }
      array[size-1]=newData; //replace final value with the new data
      return;
    }

    float getAvgInRollingAvg(float array[], int size){
      float sum = 0;
      return ((std::accumulate(array,array+size,sum))/size);
    }

    void inputNewData(float newdata, char datatype){
      switch (datatype) { 
        case 'X': shiftArray(newdata,acclRaw[0],ROLLING_AVG_LEN) ; break;
        case 'Y': shiftArray(newdata,acclRaw[1],ROLLING_AVG_LEN) ; break;
        case 'Z': shiftArray(newdata,acclRaw[2],ROLLING_AVG_LEN) ; break;
        case 'x': shiftArray(newdata,gyroRaw[0],ROLLING_AVG_LEN) ; break;
        case 'y': shiftArray(newdata,gyroRaw[1],ROLLING_AVG_LEN) ; break;
        case 'z': shiftArray(newdata,gyroRaw[2],ROLLING_AVG_LEN) ; break;
        case 'b': shiftArray(newdata,altitudeRaw,ROLLING_AVG_LEN) ; break;
      }
    }
    float recieveNewData(char datatype){
      switch (datatype) {
        case 'X': return getAvgInRollingAvg(acclRaw[0],ROLLING_AVG_LEN) ; break;
        case 'Y': return getAvgInRollingAvg(acclRaw[1],ROLLING_AVG_LEN) ; break;
        case 'Z': return getAvgInRollingAvg(acclRaw[2],ROLLING_AVG_LEN) ; break;
        case 'x': return getAvgInRollingAvg(gyroRaw[0],ROLLING_AVG_LEN) ; break;
        case 'y': return getAvgInRollingAvg(gyroRaw[1],ROLLING_AVG_LEN) ; break;
        case 'z': return getAvgInRollingAvg(gyroRaw[2],ROLLING_AVG_LEN) ; break;
        case 'b': return getAvgInRollingAvg(altitudeRaw,ROLLING_AVG_LEN) ; break;
        
      }
      return 0;
    }
};
roll roller;
void setup() {

  

  Serial.begin(115200);
  pinMode(25, OUTPUT);
  digitalWrite(25,HIGH);
  delay(4000);
  digitalWrite(25,LOW);
  delay(4000);
  digitalWrite(25,HIGH);
  Serial.println("hello!!!!");



  sw1.attach(SW1,INPUT_PULLUP);
  sw1.interval(2);
  sw2.attach(SW2,INPUT_PULLUP);
  sw2.interval(2);

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
  Serial.println(":))");
sox.configInt1(0,1,0);
sox.configIntOutputs(1,1);
Serial.println(":)))");
  if (!lps.begin_SPI(LPS_CS)) {
    while(1){
      Serial.println("nooooooo LPS didnt work");
      delay(100);
    }
  }
  Serial.println(":))))");
lps.configureInterrupt(1,1,1);
Serial.println(":))))");
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
  /*
  pinMode(LSM_INT_1,INPUT_PULLUP);
  pinMode(LSM_INT_2,INPUT_PULLUP);
  pinMode(LPS_INT,INPUT_PULLUP);
  Serial.println(":))))");
  attachInterrupt(LSM_INT_1,imuDatRdy, FALLING);
  Serial.println(":))))))))");
  attachInterrupt(LPS_INT,baroDatRdy,FALLING);
  Serial.println(":))))))");
  noInterrupts();
  interrupts();*/
  timebefore=micros();
}

void loop() {
  /*
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
  */

    sox.getEvent(&accel, &gyro, &temp);

  /* display accl vs integrated accl */
  Serial.print(accel.acceleration.x);
  roller.inputNewData(accel.acceleration.x,'X');
  Vx+=roller.recieveNewData('X')*((double)(micros()-timebefore)/1000000);
  Serial.print(",");
  Serial.print(Vx);
  Serial.print(",");
  Serial.print(accel.acceleration.y);
  roller.inputNewData(accel.acceleration.y,'Y');
  Vy+=roller.recieveNewData('Y')*((double)(micros()-timebefore)/1000000);
  Serial.print(",");
  Serial.print(Vy);
  Serial.print(",");
  Serial.print(accel.acceleration.z);
  roller.inputNewData(accel.acceleration.z,'Z');
  Vz+=(((roller.recieveNewData('Z')-10)*((double)(micros()-timebefore)/1000000)));
  Serial.print(",");
  Serial.print(Vz);
  Serial.print("\n");

  /* Display the results (rotation is measured in rad/s) */
  /*
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println("");
  
  lps.getEvent(&pressure, &temper);// get pressure
  Serial.println("lps dat:");
  Serial.print("Temperature: ");Serial.print(temper.temperature);Serial.println(" degrees C");
  Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa");
  Serial.print(baroInts);
  Serial.print(" interrupts triggered\n"); 
*/
timebefore=micros();
}






void imuDatRdy(){
 //display imu dats

imuInts++;
  sox.getEvent(&accel, &gyro, &temp);
Serial.println("lsm dat:");
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

  baroInts++;

  lps.getEvent(&pressure, &temper);// get pressure
  Serial.println("lps dat:");
  Serial.print("Temperature: ");Serial.print(temper.temperature);Serial.println(" degrees C");
  Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa");
  Serial.print(baroInts);
  Serial.print(" interrupts triggered\n");
}