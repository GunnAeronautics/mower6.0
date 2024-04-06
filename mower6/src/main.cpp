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
#include <Adafruit_BMP3XX.h>
using namespace std;
#define ISBMP false
#define TARGET_HEIGHT 180.0//in meters
int state=-1; //**DO NOT HAVE STATE AS -1 FOR ACTUAL LAUNCH** actual launch state is 0
float srvOffset = 0;


#define GRAVITY 9.807//m/s^2
#define ALT_THRESH 50

#define IMU_DATA_RATE MPU6050_CYCLE_40_HZ
#define BAROMETER_DATA_RATE LPS22_RATE_75_HZ

#define ROLLING_AVG_LEN 7

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


unsigned long initialSweepMillis = 0;
//equation of line variables for linear regression
volatile double m;
volatile double b;
double correl;
bool ifsetup = true;
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
  //Objects
File dataFile;
  //Debouncing for switches
Bounce sw1,sw2;

Servo funny;
Adafruit_LPS22 baro;
Adafruit_MPU6050 mpu;

Adafruit_BMP3XX bmp; //adafruit you're terrible
Servo srv;

double groundTemperature;
double groundPressure;
double altVTemp;
int globalIndex = 0;//current index being written by rolling avg class-written in loop
double baroRaw[ROLLING_AVG_LEN];
double baroTemperRaw[ROLLING_AVG_LEN];
double IMUAccelRaw[ROLLING_AVG_LEN];
double pressureVelocity[ROLLING_AVG_LEN];
double baroAlt[ROLLING_AVG_LEN];
double kVals[ROLLING_AVG_LEN];
double input_times[ROLLING_AVG_LEN];
double flapAngles[ROLLING_AVG_LEN];
float srvPos;
void shiftArray(double newData, double *array, int index) {
  array[index] = newData; // replace index value with the new data
  return;
}
double getRollingAvg(double array[]){
  double sum = 0;
  return ((std::accumulate(array,array+ROLLING_AVG_LEN,sum))/ROLLING_AVG_LEN);
}
void setInputTime(){
  shiftArray((double)millis(),input_times,globalIndex);
}
void inputNewData(double newdata, char datatype){//b = pressure, t=temp, a=imuaccl, v=baroVelocity, A= altitude, k=K, f=flap angle
  switch (datatype) { 
    case 'b':shiftArray(newdata, baroRaw, globalIndex);break;
    case 't':shiftArray(newdata, baroTemperRaw, globalIndex);break;
    case 'a':shiftArray(newdata, IMUAccelRaw, globalIndex);break;
    case 'v':shiftArray(newdata, pressureVelocity, globalIndex);break;
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
    case 'v':return getRollingAvg(pressureVelocity);break;
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
  case 'v':return pressureVelocity[globalIndex];break;
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

float radiansToDegrees(float angle){
  return angle*180/PI;
}
float DegreesToRadians(float angle){
  return angle/180*PI;
}
float flapAngleToServoAngle(float flapAngle){
  flapAngle = DegreesToRadians(flapAngle);
  if (flapAngle > 0.983){
    flapAngle = 1;
  }
  else if (flapAngle < 0){
    flapAngle = 0.05;
  }
  return radiansToDegrees(LUT[int(round(flapAngle*20))]);
  }

double pressToAlt(double pres){ //returns alt (m) from pressure in hecta pascals and temperature in celsius - FULLY TESTED
  return (double)(((273.0+groundTemperature)/(-.0065))*((pow((pres/groundPressure),((8.314*.0065)/(GRAVITY*.02896))))-1.0)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}
int linreg(volatile double x[], volatile double y[], volatile double *m, volatile double *b, double *r) { // stolen code
  float sumx = 0;                           // sum of x
  float sumx2 = 0;                          // sum of x**2
  float sumxy = 0;                          // sum of x * y
  float sumy = 0;                           // sum of y
  float sumy2 = 0;                          // sum of y**2
  for (int i = 0; i < ROLLING_AVG_LEN; i++) {
    sumx += x[i];
    sumx2 += pow(x[i],2);
    sumxy += x[i] * y[i];
    sumy += y[i];
    sumy2 += pow(y[i],2);
  }

  float denom = (ROLLING_AVG_LEN * sumx2 - pow(sumx,2));
  if (denom == 0) { // vertical line
    // singular matrix. can't solve the problem.
    // *m = 0;
    // *b = 0;
    if (r)
      *r = 0;
    return 1;
  }

  *m = (ROLLING_AVG_LEN * sumxy - sumx * sumy) / denom;
  *b = (sumy * sumx2 - sumx * sumxy) / denom;
  if (r != NULL) {
    *r = (sumxy - sumx * sumy / ROLLING_AVG_LEN) / // compute correlation coeff
         sqrt((sumx2 - sqrt(sumx) / ROLLING_AVG_LEN) *
              (sumy2 - sqrt(sumy) / ROLLING_AVG_LEN));
  }
  return 0;
}
double dummyB,dummyCorrel;//dump locations for lin reg on velocity & acceleration
double accly;
void dataStuff(){
  
  sensors_event_t temper;
  sensors_event_t pressure;
  sensors_event_t a, g, temperate;
  
	mpu.getEvent(&a, &g, &temperate);
  accly=a.acceleration.y;
  baro.getEvent(&pressure, &temper);
  inputNewData(pressure.pressure,'b');
  inputNewData(temper.temperature,'t');
//SOMEONE TEST PLS THIS WAS WRITTEN WHILE WALKING IN AN AIRPORT PLS
  //inputNewData( bmp.readPressure(),'b');
  // inputNewData(bmp.readTemperature(),'t');
  //#endif

   inputNewData(a.acceleration.y,'a');//add magnitude of acceleration - no gravity adjustment
   setInputTime();

  if (!ifsetup){
    inputNewData(pressToAlt( recieveRawData('b')),'A');
  }
  else{
    inputNewData(0,'A');
  }
  linreg( input_times, baroAlt,&altVTemp,&dummyB, &dummyCorrel);
  inputNewData(altVTemp,'v');
  linreg( input_times, pressureVelocity,&altitudeA,&dummyB, &dummyCorrel);
  
  
  //inputting k via a/v^2
   inputNewData( recieveRawData('a')/(pow( recieveRawData('v'),2)),'k');
}

double predictApogee (double k,double v,double alt){ // finds the distance to max alt, takes v and k - ONLY VALID FOR COASTING, assumes raw imu acceleration values
return alt + log((k*v*v/GRAVITY)+1)/(2.0*k);
}

float inverseApogee(double desiredApogee, double v) { // Working & Tested
  float searchRangeH = 20;
  float searchRangeL = 0;//SEARCH RANGE IS 0<m<20
  float mid;
  for (int i = 0; i < 20; i++) {
    mid = (searchRangeL + searchRangeH) / 2.0;
    float prediction = predictApogee(mid, v, mid*v*v);

    if (prediction < desiredApogee) { // to the left of desired
      searchRangeH = mid;
    } else if (prediction > desiredApogee) {
      searchRangeL = mid;
    }
  }
  return mid;
}
#define FUNKYLOGISTIC true //if this is false then the program will use a simple method rather than a curve if the correlation is low betweek k and flap angle
double finalcalculation (){ //returns FLAP angle, not servo, currently simple, could be complicated
  float currK= recieveRolledData('k'); float currV= recieveRolledData('v');
  float altPrediction = predictApogee(currK,currV, recieveRolledData('A'));
  float altError=altPrediction-TARGET_HEIGHT;//the amount of added error to get perfect target height
  //simple control system, could do some cooler stuff if tested RELIES ON DOWNSTREAM VARIABLE CLIPPING TO PREVENT OOR

  if(altError<(-TARGET_HEIGHT*.01)){ //just reduce the flap angle if under shooting
    return flapAngles[globalIndex]-1;
  } 

  if(!linreg(kVals,flapAngles,&m,&b,&correl)){
    Serial.println("linear regression failed, just increasing flap");
    return flapAngles[globalIndex]+1;
  } //find flapAngle/k
    float desiredK=inverseApogee(TARGET_HEIGHT,recieveRawData('v'));
  if(pow(correl,2)<.4||isinf(m*desiredK+b)){ //if correlation is low (and overshooting) just use a logistic function to adjust flap angle- https://www.desmos.com/calculator/emvbfgtl5t - might cause some oscillations
    #if FUNKYLOGISTIC==true
    if(altError>0){return flapAngles[globalIndex]+10.0/(1+exp(.2*(-abs(altError)+20)));}
    else{return flapAngles[globalIndex]-10.0/(1+exp(.2*(-abs(altError)+20)));}
    return 90.0/(1+exp(.2*(-abs(altError)+20)));
    #else //switching between logistic method and a basic addition method
    return flapAngles[globalIndex]+1*(altError>0)-1*(altError<=0); 
    #endif
  }
    
  //find k needed

  return m*desiredK+b; //use regression line to find flap angle that should theoretically give us optimal values
}

void writeSDData(){ 
  String dataString = (String)millis() + ',' +
                      (String) (recieveRolledData('A')*1000)+ ',' + //alt
                      (String) (recieveRolledData('v')*1000) + ',' + //velocity - baro derived
                      (String) (recieveRolledData('a')*1000) + ',' +//accel - imu derived
                      //(String)(altitudeA*1000) + ',' +//accl baro
                      (String)predictApogee( recieveRolledData('k'), recieveRolledData('v'),recieveRolledData('A')) + ',' + //apogee prediction
                      (String)srvPos + ',' +
                      (String)recieveRawData('f') + ',' + //flap angle
                      (String)state+','+(String)correl;
                      
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


float currentFlapAngle;
void setup() {

  Serial.begin(115200);
  delay(7000);
  Serial.println("haiiiiii >_< :3");
  pinMode(LED_BUILTIN,OUTPUT);
      digitalWrite(LED_BUILTIN,HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN,LOW);
  // Ensure the SPI pinout the SD card is connected to is configured properly

  SPI.setRX(SPI_RX);
  SPI.setTX(SPI_TX);
  SPI.setSCK(SPI_SCLK);
  #if !ISBMP //switches between lps22 and BMP388 DONT MISS THIS ONE TONY PLS :)
  Serial.print("Barometer initialized, initialization bool = ");Serial.println(baro.begin_SPI(BARO_CS));
  baro.setDataRate(BAROMETER_DATA_RATE);
  #endif

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
        dataFile.println("time,altitude,baro velocity,imu acceleration,apogee predicted,servo pos,flap angle,state");
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
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.println("MPU6050 chip found");
  #if ISBMP
  bmp.begin_I2C(119U,&Wire); //david deal with this
  bmp.setOutputDataRate(BMP_DATA_RATE);
  #endif
  mpu.setCycleRate(IMU_DATA_RATE);

  delay(1000);
  Serial.println("waiting for launch");

  for (int i=0; (i < ROLLING_AVG_LEN); i++){
    dataStuff();//zeros a bunch of stuff
    delay(100);
    globalIndex++;
    globalIndex%=ROLLING_AVG_LEN;
  }
  
  groundTemperature =  recieveRolledData('t'); //used as ground reference
  groundPressure =  recieveRolledData('b'); 
  Serial.println("Zeroed Altitude");
  srv.attach(SERVO_ONE); //closest to board
  
  delay(1000);
  srv.write(3*(flapAngleToServoAngle(0)+ srvOffset));
  delay(3000);
  for (int i=0; i < 30; i++){
    srv.write(3*(flapAngleToServoAngle(i*2)+srvOffset));
    delay(25);
  }
  delay(3000);
  for (int i=30; i > 0; i--){
    srv.write(3*(flapAngleToServoAngle(i*2)+srvOffset));
    Serial.println(i);
    delay(25);
  }
  ifsetup = false;  
  
}


uint8_t consecMeasurements = 0; //used for state transitions
int timer;
void loop() {
  deltaT = (millis() - lastT)/1000.0;//1000;// in seconds
  lastT = millis();// in seconds
  dataStuff();
  
  switch (state){

   case -1: //debug
    writeSDData();
    if (timer < millis()){

    Serial.print("Baro:");
    Serial.print( recieveRawData('b'));
    Serial.print(" DeltaT:");
    Serial.print(deltaT*1000);
    Serial.print(" Yaccel:");
    Serial.print(altitudeA);
    Serial.print(" Yaccel2:");
    Serial.print( recieveRolledData('a'));
    Serial.print(" Yvel:");
    Serial.print( recieveRolledData('v'));
    Serial.print(" Yalt:");
    Serial.println(recieveRawData('A'));
    timer = millis() + 300;
    }
   break;

  case 0://on launch pad
    if (recieveRolledData('A')> ALT_THRESH&&accly<0){ 
      state = 1;//rocket has gone off the launch pad
    }
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
    currentFlapAngle=flapAngleToServoAngle(finalcalculation());
    
    writeSDData();
    break;

  
  case 2://falling
  if (consecMeasurements == 10){
        state = 3;
        consecMeasurements=0;
      }
    else if ((recieveRolledData('v') > -0.1) && (recieveRolledData('v') < 0.1)){
        consecMeasurements++;
      }
    else{
        consecMeasurements = 0;
      }
    currentFlapAngle=flapAngleToServoAngle(0);
    
    writeSDData();
    delay(150);
    break;

  case 3://on the ground, do nothing
    digitalWrite(DEBUG_LED,1);
    delay(200);
    digitalWrite(DEBUG_LED,0);
    delay(200);
    break;
  }
  srvPos = currentFlapAngle;
  srv.write(2*(srvPos+srvOffset));//idk why 2x srv pos but Im not judging
  inputNewData(currentFlapAngle,'f');
  globalIndex++;
  globalIndex%=ROLLING_AVG_LEN;
  
}