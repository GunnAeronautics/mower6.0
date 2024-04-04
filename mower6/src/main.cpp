#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Bounce2.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <numeric>
#include <math.h>
using namespace std;

#define TARGET_HEIGHT 180//in meters
#define GRAVITY 9.807//m/s^2
#define ALT_THRESH 25

#define IMU_DATA_RATE MPU6050_CYCLE_40_HZ
#define BAROMETER_DATA_RATE LPS22_RATE_75_HZ

#define ROLLING_AVG_LEN 7

#define SRV_SWEEP_TIME 2500//in millis

//Pin defs 
#define DEBUG_LED 25
#define SD_CS 5
#define BARO_CS 6
#define SPI_SCLK 18
#define SPI_TX 19 
#define SPI_RX 20 
#define SERVO_ONE 13
#define SERVO_TWO 12
#define SERVO_THREE 11


  int state=-1; //**DO NOT HAVE STATE AS -1 FOR ACTUAL LAUNCH**


  unsigned long lastBeepTime = 0; //the last time the beep happened during case 4 (beep)

  float srvPos; //servo position array


  uint8_t consecMeasurements = 0; //this variable should never be greater than 4. Defined as 8-bit integer to save memory

  unsigned long initialSweepMillis = 0;
//equation of line variables for linear regression
volatile double m;
volatile double b;
double correl;

float LUT[21] = {//20x 0.05 radian increments starting from fully retracted final is 0.983
1.087//degree in radians of servo for fully retracted flaps
,1.073
,0.966
,0.912
,0.858
,0.805
,0.751
,0.697
,0.644
,0.590
,0.537
,0.483
,0.429
,0.376
,0.322
,0.268
,0.215
,0.161
,0.107
,0.054
,0.0};
float LUTinv[21]={ //input = servo position (offset to fully [EXTENDED (change if neccisary)]), output = flap angle for servo position (scaled between 0-90 degrees) TODO

};
  //Objects
File dataFile;
  //Debouncing for switches
Bounce sw1,sw2;

Servo funny;
Adafruit_LPS22 baro;
Adafruit_MPU6050 mpu;

Servo srv;

double originalTemper;
double originalBaro;
//rolling average
int globalIndex = 0;//current index being written by rolling avg class-written in loop

    double baroRaw[ROLLING_AVG_LEN];
    double baroTemperRaw[ROLLING_AVG_LEN];
    double IMUAccelRaw[ROLLING_AVG_LEN];
    double yBaroVelRaw[ROLLING_AVG_LEN];
    double baroAlt[ROLLING_AVG_LEN];
    double kVals[ROLLING_AVG_LEN];
    double input_times[ROLLING_AVG_LEN];
    double flapAngles[ROLLING_AVG_LEN];
    void shiftArray(double newData, double *array, int index) {
      array[index] = newData; // replace index value with the new data
      return;
    }

    double getRollingAvg(double array[]){
      double sum = 0;
      return ((std::accumulate(array,array+ROLLING_AVG_LEN,sum))/ROLLING_AVG_LEN);
    }
    void setInputTime(){
      shiftArray(millis(),input_times,globalIndex);
    }
    void inputNewData(double newdata, char datatype){//b = pressure, t=temp, a=imuaccl, v=baroVelocity, A= altitude, k=K, f=flap angle
      switch (datatype) { 
        case 'b':shiftArray(newdata, baroRaw, globalIndex);break;
        case 't':shiftArray(newdata, baroTemperRaw, globalIndex);break;
        case 'a':shiftArray(newdata, IMUAccelRaw, globalIndex);break;
        case 'v':shiftArray(newdata, yBaroVelRaw, globalIndex);break;
        case 'A':shiftArray(newdata, baroAlt, globalIndex);break;
        case 'k':shiftArray(newdata, kVals, globalIndex);break;
        case 'f':shiftArray(newdata, kVals, globalIndex);break;
      }
    }
    double recieveRolledData(char datatype){//b = pressure, t=temp, a=imuaccl, v=baroVelocity, A= altitude
      switch (datatype) {
        case 'b':return getRollingAvg(baroRaw);break;
        case 't':return getRollingAvg(baroTemperRaw);break;
        case 'a':return getRollingAvg(IMUAccelRaw);break;
        case 'v':return getRollingAvg(yBaroVelRaw);break;
        case 'A':return getRollingAvg(baroAlt);break;
        case 'k':return getRollingAvg(kVals);break;
        case 'f':return getRollingAvg(flapAngles);break;
      }
      return 0;
    }
    double recieveRawData(char datatype) { //b = pressure, t=temp, a=imuaccl, v=baroVelocity, A= altitude
      switch (datatype) {
      case 'b':return baroRaw[globalIndex];break;
      case 't':return baroTemperRaw[globalIndex];break;
      case 'a':return IMUAccelRaw[globalIndex];break;
      case 'v':return yBaroVelRaw[globalIndex];break;
      case 'A':return baroAlt[globalIndex];break;
      case 'k':return kVals[globalIndex];break;
      case 'f':return flapAngles[globalIndex];break;
    }
    return 0;
  }


String fname="datalog.csv";

unsigned long lastT = millis();
volatile double deltaT;


volatile double realAccel[3];
volatile double gyro[3];
double altitudeA;
double dummyB,dummyCorrel;//useless dump locations for lin reg on velocity & acceleration



float radiansToDegrees(float angle){
  return angle*180/PI;
}
float DegreesToRadians(float angle){
  return angle/180*PI;
}
float servoAngleToFlapAngle(float servoAngle){ //TODO
return LUTinv[(int)(servoAngle*21/90)];
}
float flapAngleToServoAngle(float flapAngle){
  flapAngle = DegreesToRadians(flapAngle);
  if (flapAngle > 0.983){
    flapAngle = 1;
  }
  else if (flapAngle < 0){
    flapAngle = 0;
  }
  return radiansToDegrees(LUT[int(round(flapAngle*20))]);
  }

double pressToAlt(double pres){ //returns alt (m) from pressure in hecta pascals and temperature in celsius - FULLY TESTED
  return (double)(((273.0+originalTemper)/(-.0065))*((pow((pres/originalBaro),((8.314*.0065)/(9.807*.02896))))-1.0)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}
int linreg(volatile double x[], volatile double y[], volatile double* m, volatile double* b, double* r){ //stolen code
  float sumx = 0;                      // sum of x     
  float sumx2 = 0;                     // sum of x**2  
  float sumxy = 0;                     // sum of x * y 
  float sumy = 0;                      // sum of y    
  float sumy2 = 0;                     // sum of y**2  
  for (int i=0;i<ROLLING_AVG_LEN;i++){ 
    sumx  += x[i];       
    sumx2 += sqrt(x[i]);  
    sumxy += x[i] * y[i];
    sumy  += y[i];      
    sumy2 += sqrt(y[i]); 
  }

  float denom = (ROLLING_AVG_LEN * sumx2 - sqrt(sumx));
  if (denom == 0) {//vertical line 
      // singular matrix. can't solve the problem.
      // *m = 0;
      // *b = 0;
    if (r) *r = 0;
      return 1;
  }

  *m = (ROLLING_AVG_LEN * sumxy  -  sumx * sumy) / denom;
  *b = (sumy * sumx2  -  sumx * sumxy) / denom;
  if (r!=NULL) {
      *r = (sumxy - sumx * sumy / ROLLING_AVG_LEN) /    //compute correlation coeff 
            sqrt((sumx2 - sqrt(sumx)/ROLLING_AVG_LEN) *
            (sumy2 - sqrt(sumy)/ROLLING_AVG_LEN));
  }
  return 0; 
}
void dataStuff(){
  sensors_event_t temper;
  sensors_event_t pressure;
  sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);

  baro.getEvent(&pressure, &temper);
   inputNewData(pressure.pressure,'b');
   inputNewData(temper.temperature,'t');
   inputNewData(sqrt(pow(a.acceleration.x,2)+pow(a.acceleration.y,2)+pow(a.acceleration.z,2)),'a');//add magnitude of acceleration - no gravity adjustment
   setInputTime();
  gyro[0] = g.gyro.x;
  gyro[1] = g.gyro.y;
  gyro[2] = g.gyro.z;
   inputNewData(pressToAlt( recieveRawData('b')),'A');
  double altVTemp;
  linreg( input_times, baroAlt,&altVTemp,&dummyB, &dummyCorrel);
   inputNewData(altVTemp,'v');
  delete &altVTemp;
  linreg( input_times, yBaroVelRaw,&altitudeA,&dummyB, &dummyCorrel);
  
  //inputting k via a/v^2
   inputNewData( recieveRawData('a')/(pow( recieveRawData('v'),2)),'k');
}

double predictApogee (double k,double v){ // finds the distance to max alt, takes v and k - ONLY VALID FOR COASTING, assumes raw imu acceleration values
return log((k*v*v/9.81)+1)/(2.0*k);
}

float inverseApogee(double desiredApogee, double v) { // Working & Tested
  float searchRangeH = 20;
  float searchRangeL = 0;//SEARCH RANGE IS 0<m<20
  float mid;
  for (int i = 0; i < 20; i++) {
    mid = (searchRangeL + searchRangeH) / 2.0;
    float prediction = predictApogee(mid, v);

    if (prediction < desiredApogee) { // to the left of desired
      searchRangeH = mid;
    } else if (prediction > desiredApogee) {
      searchRangeL = mid;
    }
  }
  return mid;
}


float finalcalculation (){ //returns FLAP angle - TODO
  float currK= recieveRawData('k'); float currV= recieveRawData('v');
  float altPrediction = predictApogee(currK,currV);

  //implement hysteresis
  if(altPrediction<TARGET_HEIGHT);
  
  //if greater use cool stuff 
}

void writeSDData(){ 
  String dataString = (String)millis() + ',' +
                      (String) recieveRawData('b') + ',' + //pressure
                      (String) recieveRawData('A')+ ',' + //alt
                      (String) recieveRawData('v') + ',' + //velocity - baro derived
                      (String) recieveRawData('a') + ',' +//accel - imu derived
                      (String)altitudeA + ',' +//accl baro
                      (String)predictApogee( recieveRolledData('k'), recieveRolledData('v')) + ',' + //apogee prediction
                      (String)srvPos + ',' +
                      (String)recieveRawData('f') + ',' + //flap angle
                      (String)state;
                      
  File dataFile = SD.open(fname, FILE_WRITE);
    // if the file is available, write to it:
  if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
  }
  
  else {
    Serial.print("error opening ");
    Serial.println(fname);
  }
}

float srvOffset = flapAngleToServoAngle(0);



void setup() {

  Serial.begin(115200);
  delay(7000);
  Serial.println("haiiiiii");
  pinMode(LED_BUILTIN,OUTPUT);
      digitalWrite(LED_BUILTIN,HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN,LOW);
  // Ensure the SPI pinout the SD card is connected to is configured properly

  SPI.setRX(SPI_RX);
  SPI.setTX(SPI_TX);
  SPI.setSCK(SPI_SCLK);

  Serial.print("Barometer initialized, initialization bool = ");Serial.println(baro.begin_SPI(BARO_CS));
  baro.setDataRate(BAROMETER_DATA_RATE);

  Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
    Serial.println("Card failed, or not present");

    while(true){
      digitalWrite(DEBUG_LED,1);
      delay(200);
      digitalWrite(DEBUG_LED,0);
      delay(200);
    }
    }
    else {

      Serial.println("SD initialized");
        delay(100);
        fname="datalog"+(String)0+".csv";

        for (int i=0; i<999&&(SD.exists(fname));i++){ //add detection if file already exists
          fname="datalog"+(String)i+".csv";
          Serial.print("tried ");
          Serial.println(fname);
        }
        File dataFile = SD.open(fname, FILE_WRITE);
        
        if (!dataFile){
          Serial.println("dat file not initialized, name = ");
          Serial.print(fname);

        } else {
          Serial.println("dat file successfully initialized, name = ");
          Serial.print(fname);
        }
        dataFile.println("time,pressure,altitude,baro velocity,imu acceleration,baro acceleration,apogee predicted,servo pos,flap angle,state");
        dataFile.close();
    }
    Serial.println("0");
    Wire.setSCL(9);
    Serial.println("1");
    Wire.setSDA(8);
    Serial.println("2");
    
  if (!mpu.begin(0x68,&Wire)) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
  mpu.setCycleRate(IMU_DATA_RATE);

  delay(1000);
  Serial.println("waiting for launch");

  for (int i=0; (i < ROLLING_AVG_LEN); i++){
    dataStuff();//zeros a bunch of stuff
    delay(100);
    globalIndex++;
    globalIndex%=ROLLING_AVG_LEN;
  }
  originalTemper =  recieveRolledData('t');
  originalBaro =  recieveRolledData('b'); 
  
  srv.attach(SERVO_ONE); //closest to board

  delay(3000);
}



void loop() { //ACTUAL CONTROL NOT FULLY IMPLEMENTED YET
  deltaT = (millis() - lastT)/1000.0;//1000;// in seconds
  lastT = millis();// in seconds
  dataStuff();
  
  switch (state){

   case -1: //debug
    writeSDData();
    while((millis()-lastT)<300){delay(1);}
    Serial.print("Baro:");
    Serial.print( recieveRawData('b'));
    Serial.print(" DeltaT:");
    Serial.print(deltaT*1000);
    
    Serial.print(" Yaccel:");
    Serial.print(altitudeA);
    Serial.print(" Yaccel2:");
    Serial.print( recieveRolledData('a'));
    Serial.print(" Yvel:");
    Serial.print( recieveRolledData('v')*100);
    Serial.print(" Yalt:");
    Serial.println(recieveRawData('A'));
   break;

  case 0://on launch pad

    if (recieveRolledData('A')> ALT_THRESH){ 
      state = 1;
      Serial.println("launch detected, beginning logging");
      
    }
    //delay(20);
    break;
  case 1: //doing stuff
    if (consecMeasurements == 3){//exit loop when the rocket is at apogee
        state = 2;
        consecMeasurements=0;
      }
    else if (recieveRolledData('v')<0){
        consecMeasurements++;
      }
    else{
        consecMeasurements = 0;
      }
    srvPos=flapAngleToServoAngle(finalcalculation());
    writeSDData();
    break;

  
  case 2:
  if (consecMeasurements == 3){
        state = 3;
        consecMeasurements=0;
        Serial.println("reached ground");
      }
    else if ( recieveRolledData('a')){
        consecMeasurements++;
      }
    else{
        consecMeasurements = 0;
      }
    writeSDData();
    delay(150);
    break;

  case 3://on the ground, do nothing
    delay(100);
    break;
  }

  srv.write(2*(srvPos+srvOffset));//idk why 2x srv pos but Im not judging
  inputNewData(servoAngleToFlapAngle(srvPos),'f');
  globalIndex++;
  globalIndex%=ROLLING_AVG_LEN;
  
}

