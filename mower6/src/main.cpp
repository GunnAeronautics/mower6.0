#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Bounce2.h>
#include <Wire.h>
#include <Servo.h>
//using adafruit's libraries

#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <numeric>
#include <math.h>
using namespace std;

#define TARGET_TIME 44.5 //in seconds
#define TARGET_HEIGHT 180//in meters
#define GRAVITY 9.807//m/s^2

#define VEL_THRESH 0
#define BURNOUT_ALITUTDE 100
#define ALT_THRESH 25 //meters above initial


#define IMU_DATA_RATE LSM6DS_RATE_104_HZ
#define BAROMETER_DATA_RATE LPS22_RATE_75_HZ


#define ROLLING_AVG_LEN 7

#define SRV_SWEEP_TIME 2500//in millis

//Pin defs 

#define BARO_INT 13
#define DEBUG_LED 25
#define SD_CS 5//chip select//correct
#define BARO_CS 6//chip select//correct
//TX = DO = MOSI, RX = DI=MISO
#define SPI_SCLK 18//correct
#define SPI_TX 19 //AKA mosi//correct
#define SPI_RX 20 //AKA miso//correct
#define SERVO_ONE 13
#define SERVO_TWO 12
#define SERVO_THREE 11
#define CTRL_SW1 26
#define CTRL_SW2 22
#define BUZZ_PIN 3 //using tone function
    //Runtime variables

  int state=-1; //**DO NOT HAVE STATE AS -1 FOR ACTUAL LAUNCH**


  unsigned long lastBeepTime = 0; //the last time the beep happened during case 4 (beep)

  float srvPos; //servo position array

  float currentFlapAngle;
  int8_t consecMeasurements = 0; //this variable should never be greater than 4. Defined as 8-bit integer to save memory
  unsigned long initialSweepMillis = 0;
//equation of line variables for linear regression
volatile double m;
volatile double b;
double correl;

volatile double flapAngleAndKTable[2][ROLLING_AVG_LEN];
volatile double lastValues[3][ROLLING_AVG_LEN];//Time, Displacement, Velocity last 10 measurements
float LUT[21] = {//20x 0.05 radian increments starting from 0 degress retracted final is 0.983
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
int globalIndex = 0;//indexxes all the rolling things 
class roll{//tested (it works)
  public:
    //float accltotal[ROLLING_AVG_LEN];
    double baroRaw[ROLLING_AVG_LEN];
    double baroTemperRaw[ROLLING_AVG_LEN];
    double yIMUAccelRaw[ROLLING_AVG_LEN];
    double yBaroVelRaw[ROLLING_AVG_LEN];

    void shiftArray(double newData, double *array, int index) {
      array[index] = newData; // replace index value with the new data
      return;
    }

    double getRollingAvg(double array[]){
      double sum = 0;
      return ((std::accumulate(array,array+ROLLING_AVG_LEN,sum))/ROLLING_AVG_LEN);
    }

    void inputNewData(double newdata, char datatype){
      switch (datatype) { 
        case 'b':shiftArray(newdata, baroRaw, globalIndex);break;
        case 't':shiftArray(newdata, baroTemperRaw, globalIndex);break;
        case 'a':shiftArray(newdata, yIMUAccelRaw, globalIndex);break;
        case 'v':shiftArray(newdata, yBaroVelRaw, globalIndex);break;
      
      
      }
    }
    double recieveData(char datatype){
      switch (datatype) {
        case 'b':return getRollingAvg(baroRaw);break;
        case 't':return getRollingAvg(baroTemperRaw);break;
        case 'a':return getRollingAvg(yIMUAccelRaw);break;
        case 'v':return getRollingAvg(yBaroVelRaw);break;
      }
      return 0;
    }
    double recieveRawData(char datatype) {
      switch (datatype) {
      case 'b':return baroRaw[globalIndex];break;
      case 't':return baroTemperRaw[globalIndex];break;
      case 'a':return yIMUAccelRaw[globalIndex];break;
      case 'v':return yBaroVelRaw[globalIndex];break;
    }
    return 0;
  }

};
roll roller;//rolling object
void buzztone (int time, int frequency);
void imuIntRoutine();
void baroIntRoutine();

String fname="datalog.csv";
volatile bool isSetUp=false; //prob being read at the same time by both cores

unsigned long lastT = millis();
volatile double deltaT;

volatile double altitude[2];
volatile double altitudeV[2];//velocity in altitude
volatile double altitudeA;//acceleration in altitude
volatile double realAccel[3];
volatile double gyro[3];
int sdCardActive = 0;//If sd card is active or not
float desiredFlapAngle;
int derivationStrategy = 0;// 0 for simple D/T, 1 for IMU acceleration,2 for linear regression size rolling avg

volatile double dummyB;//useless dump locations for lin reg on velocity & acceleration
double dummyCorrel;//useless dump locations for lin reg on velocity & acceleration



float radiansToDegrees(float angle){
  return angle*180/PI;
}
float DegreesToRadians(float angle){
  return angle/180*PI;
}
//   https://www.desmos.com/calculator/q2bzbwfqkn
float flapAngleToServoAngle(float flapAngle){ //flap angle in degrees, returns srv angle in degrees
  flapAngle = DegreesToRadians(flapAngle);
  if (flapAngle > 0.983){
    flapAngle = 1;
  }
  else if (flapAngle < 0){
    flapAngle = 0;
  }
  return radiansToDegrees(LUT[int(round(flapAngle*20))]);//-1 cuz base 0
  }
  //lookup table bruh
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
void dataStuff(){//does all of the data filtering etc
  sensors_event_t temper;
  sensors_event_t pressure;
  sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);

  baro.getEvent(&pressure, &temper);
  roller.inputNewData(pressure.pressure,'b');
  roller.inputNewData(temper.temperature,'t');
  roller.inputNewData(sqrt(pow(a.acceleration.x,2)+pow(a.acceleration.y,2)+pow(a.acceleration.z,2))-GRAVITY,'a');//assume the rocket is not accelerating sideways


  realAccel[0] = a.acceleration.x;
  realAccel[1] = a.acceleration.y;
  realAccel[2] = a.acceleration.z;
  gyro[0] = g.gyro.x;
  gyro[1] = g.gyro.y;
  gyro[2] = g.gyro.z;

  if (derivationStrategy = 0){
    altitude[1] = altitude[0];
    altitude[0] = pressToAlt(roller.recieveData('b'));
  
    altitudeV[1] = altitudeV[0];
    roller.inputNewData((altitude[0]-altitude[1])/deltaT,'v');
    altitudeV[0] = roller.recieveData('v');
    altitudeA = (altitudeV[1]-altitudeV[0])/deltaT;//bruh second degree derivations go crazy
  }
  else if (derivationStrategy = 1){
    altitude[1] = altitude[0];
    altitude[0] = pressToAlt(roller.recieveData('b'));
  
    altitudeV[1] = altitudeV[0];
    altitudeV[0] = (altitude[0]-altitude[1])/deltaT;
    altitudeA = roller.recieveRawData('a');
  
  }
  
  else{
    altitude[0] = pressToAlt(roller.recieveData('b'));
    lastValues[0][globalIndex] = lastT;
    lastValues[1][globalIndex] = altitude[0];
    linreg(lastValues[0],lastValues[1],&altitudeV[0],&dummyB, &dummyCorrel);//velocity is the B
    lastValues[2][globalIndex] = altitudeV[0];
    linreg(lastValues[0],lastValues[2],&altitudeA,&dummyB, &dummyCorrel);
  }
  
}
double getPastK(double accel,double v){//acceleration without gravity
  return ((accel)/pow(v,2));//check equation pls
}
double predictApogee (double k,double v){ // finds the distance to max alt, takes v and k - ONLY VALID FOR COASTING
//returns the distance from now to the final predicted apogee
return log((v*v*k/9.81)+1)/(2.0*k);
} //https://www.rocketmime.com/rockets/qref.html for range equation
float predictApogee2 (double d, double v, double a){
  //TARGET_HEIGHT - d

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
    //cout << searchRangeL << '_' << searchRangeH << '\n';
  }
  return mid;
}

float getDesiredFlapAngle(volatile float m, volatile float b, float desiredk){//gets servo best angle from m, b, and desired K
  return (asin(sqrt(desiredk - b)/(m)));//in radians
}
float getAngleFactor(float theta){//in radians UNUSED!!!
  return (pow(sin(theta),2));//allows linear regression to work for angles linearly
}
float finalcalculation (){ //returns PREOFFSET angle for servo - servo offset handled at end of loop
//lin regress - least square method
if(!linreg(flapAngleAndKTable[0],flapAngleAndKTable[1],&m,&b,&correl)){ //if linear regression is succesfull (not vertical line)
  float ktarget = inverseApogee(TARGET_HEIGHT - ((altitude[0]+altitude[1])/2.0), roller.recieveData('v'));
  return getDesiredFlapAngle(m,b,ktarget);
} else {
  Serial.println("lin reg failed");
  //if(predictApogee())
  //float ktarget = inverseApogee(TARGET_HEIGHT - (altitude[0]+altitude[1])/2.0, (altitudeV[0]+altitudeV[1])/2.0); //still hold current method because linear regression wont change m or b
  return 10.0; 
}
}

void writeSDData(){  // FULLY TESTED
  if (sdCardActive == 1){
  String dataString = (String)millis() + ',' +

                      (String)roller.recieveRawData('b') + ',' +
                      (String)altitude[0]+ ',' +
                      (String)altitudeV[0] + ',' +
                      (String)realAccel[0] + ',' +//accelX
                      (String)altitudeA + ',' +//accelY
                      (String)realAccel[2] + ',' +//accelZ
                      (String)predictApogee(getPastK(roller.recieveData('v'),altitudeA),roller.recieveData('v')) + ',' +//predict the apogee
                      (String)srvPos + ',' +
                      (String)currentFlapAngle + ',' +
                      (String)state;
                      //(String)gyro[0]+ ',' +//gyroX
                      ////(String)gyro[1]+ ',' +//gyroY
                      //(String)gyro[2];//gyroZ
                      
  File dataFile = SD.open(fname, FILE_WRITE);
    // if the file is available, write to it:
  if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
  }
  
  else {
    //Serial.print(" error opening ");
    Serial.println(fname);
  }
  }
}
float srvOffset = flapAngleToServoAngle(0); //TO CHANGE
void setup() {
  // put your setup code here, to run once:
   /* //Comment out to activate UART messages
  Serial.setRX(1);
  Serial.setTX(0);
  // */
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
    sdCardActive = 0;
    }
    else {
      sdCardActive = 1;
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
        dataFile.println("time,baro,altitude,altVel,accelX,altAccel,accelZ,predictedApogee,srvPos,flapPos,state,gyroX,gyroY,gyroZ");
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
  Serial.println("a");
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  Serial.println("b");
  mpu.setMotionDetectionThreshold(1);

  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
Serial.println("c");
  delay(1000);
  isSetUp=true;
  Serial.println("waiting for launch");

  for (int i=0; (i < ROLLING_AVG_LEN); i++){
    dataStuff();//zeros a bunch of stuff
    delay(100);
    globalIndex++;
    globalIndex%=ROLLING_AVG_LEN;
  }
  originalTemper = roller.recieveData('t');
  originalBaro = roller.recieveData('b'); 
  
  srv.attach(SERVO_ONE); //closest to board
 // delay(3000);

  /*
  while (true){
//srv.write(180);
  for (int i=0; i<180;i++){
    srv.write(i);
    delay(5);
  }
    //srv.write();
    delay(500);
  }
  //*/
  delay(3000);
}


int timer;

void loop() {
  deltaT = (millis() - lastT)/1000.0;//1000;// in seconds
  lastT = millis();// in seconds
  dataStuff();// does all the data shit
  
  switch (state){

   case -1: //debug
    writeSDData();
    if (timer < millis()){
    Serial.print("Baro:");
    Serial.print(roller.recieveRawData('b'));
    Serial.print(" DeltaT:");
    Serial.print(deltaT*1000);
    
    Serial.print(" Yaccel:");
    Serial.print(altitudeA);
    Serial.print(" Yaccel2:");
    Serial.print(roller.recieveData('a'));
    Serial.print(" Yvel:");
    Serial.print(roller.recieveData('v')*100);
    Serial.print(" Yalt:");
    Serial.println(altitude[0]);
    timer = millis() + 300;
    }
    //delay(300);

   break;

  case 0://on launch pad

    if (altitude[0] > 6){ //acceleration greater than 10m/s^2
      state = 2;
      Serial.println("launch detected, beginning logging");
      pinMode(LED_BUILTIN,OUTPUT);
      digitalWrite(LED_BUILTIN,HIGH);
      
    }
    //delay(20);
    break;


  // put your main code here, to run repeatedly:
  
  case 1://accelerating
    if (consecMeasurements >= 3){//waiting for 3 consecutive measurements with alt above threshold
        state = 2;
        consecMeasurements=0;
      }
    else if ((altitudeA < 10) && (altitude[0] > 30)){
        consecMeasurements++;
      }
    else{
        consecMeasurements = 0;
      }
    writeSDData();
    break;

  
  case 2:
    //*
    //srvPos = flapAngleToServoAngle(45,&currentFlapAngle );//FIX LATER currently do not have energy to test srv offsets
    //currentFlapAngle = finalcalculation();
    //srvPos = flapAngleToServoAngle(currentFlapAngle);
    //srvPos = finalcalculation();
    //Serial.println(roller.recieveRawData('y'));
    //Serial.print(' ');
    //Serial.print(altitude[0]);
    // */
    /*
    if(altitude[0]<TARGET_HEIGHT){
      srvPos = 180/(1+pow(exp(1),-((1/(TARGET_HEIGHT-altitude[0]))-2)));
    } else{
      srvPos=flapAngleToServoAngle(70);
    }
    //*/
    if (consecMeasurements == 3){//exit loop when the rocket is at appogee
        state = 3;//fix later
        consecMeasurements=0;
        //Serial.println("apogee reached");
      }
    else if (altitudeV[0]<0){
        consecMeasurements++;
      }
    else{
        consecMeasurements = 0;
      }
    writeSDData();
    break;

  
  case 3://freefall -- dont write anything to save data
  if (consecMeasurements == 3){//exit loop when the rocket is at appogee
        state = 4;//fix later
        consecMeasurements=0;
        Serial.println("reached ground");
      }
    else if ((altitudeV[0] > -0.2) or (altitudeV[0] < 0.2)){//normal force acting
        consecMeasurements++;
      }
    else{
        consecMeasurements = 0;
      }
    writeSDData();
    delay(150);
    break;

  case 4://on the ground, do nothing
    delay(100);
    break;
  }

  srv.write(2*(srvPos+srvOffset));//idk why 2x srv pos but Im not judging
  globalIndex++;
  globalIndex%=ROLLING_AVG_LEN;
}

