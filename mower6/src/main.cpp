#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Bounce2.h>
#include <Servo.h>
//using adafruit's libraries

#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <numeric>
#include <math.h>

#define TARGET_TIME 44.5 //in seconds
#define TARGET_HEIGHT 250//in meters


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

#define PI 3.14159
    //Runtime variables
  int state=0;
  unsigned long lastBeepTime = 0; //the last time the beep happened during case 4 (beep)

  float srvPos; //servo position array
  float srvOffsets = 0;

  int8_t consecMeasurements = 0; //this variable should never be greater than 4. Defined as 8-bit integer to save memory
  unsigned long initialSweepMillis = 0;
//equation of line variables for linear regression
volatile float m;
volatile float b;
float correl;

volatile float flapAngleAndKTable[2][ROLLING_AVG_LEN];

  //Objects
File dataFile;
  //Debouncing for switches
Bounce sw1,sw2;


Adafruit_LPS22 baro;

Servo srv;

double originalTemper;
double originalBaro;
//rolling average

class roll{//tested (it works)
  public:
    //float accltotal[ROLLING_AVG_LEN];
    double baroRaw[ROLLING_AVG_LEN];
    double baroTemperRaw[ROLLING_AVG_LEN];
    int baroIndex = 0;

    void shiftArray(double newData, double *array, int index) {
      array[index] = newData; // replace index value with the new data
      return;
    }

    float getRollingAvg(double array[]){
      float sum = 0;
      return ((std::accumulate(array,array+ROLLING_AVG_LEN,sum))/ROLLING_AVG_LEN);
    }

    void inputNewData(double newdata, char datatype){
      switch (datatype) { 
        case 'b':shiftArray(newdata, baroRaw, baroIndex);break;
        case 't':shiftArray(newdata, baroTemperRaw, baroIndex);break;
      
      }
    }
    float recieveData(char datatype){
      switch (datatype) {
        case 'b':return getRollingAvg(baroRaw);break;
        case 't':return getRollingAvg(baroTemperRaw);break;
      }
      return 0;
    }
    float recieveRawData(char datatype) {
      switch (datatype) {
      case 'b':return baroRaw[baroIndex];break;
      case 't':return baroTemperRaw[baroIndex];break;
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
float altitude[2];
unsigned long lastT = millis();
float deltaT;
double altitudeV[2];//velocity in altitude
double altitudeA;//acceleration in altitude
float sdCardActive = 0;
void writeSDData(){ 
  if (sdCardActive == 1){
  String dataString = (String)millis() + ',' +
                      //(String)acclRaw[0] + ',' +
                      //(String)acclRaw[1]  + ',' +
                      //(String)acclRaw[2] + ',' +
                      //(String)gyroRaw[0] + ',' +
                      //(String)gyroRaw[1] + ',' +
                      //(String)gyroRaw[2] + ',' +
                      (String)roller.recieveRawData('b') + ',' +
                      (String)roller.recieveRawData('t')+ ',' +
                      (String)altitude[0]+ ',' +
                      (String)altitudeV[0] + ',' +
                      (String)altitudeA + ',' +
                      (String)state;
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
double pressToAlt(double pres){ //returns alt (m) from pressure in hecta pascals and temperature in celsius
  return (double)(((273+originalTemper)/(-.0065))*((pow((pres/originalBaro),((8.314*.0065)/(9.807*.02896))))-1)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}
void setup() {
  
  // put your setup code here, to run once:
  
  //Serial.setRX(1);
  //Serial.setTX(0);

  Serial.begin(115200);
  delay(7000);
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
    // don't do anything more:
    //while(true){}
    //return;
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
        dataFile.println("time,baro,temperature,altitude,altVel,altAccel,state");
        dataFile.close();
    }
  
  
  delay(1000);
  isSetUp=true;
  Serial.println("waiting for launch");

  for (int i=0; (i < ROLLING_AVG_LEN); i++){
    sensors_event_t temper;
    sensors_event_t pressure;
    baro.getEvent(&pressure, &temper);
    roller.inputNewData(pressure.pressure,'b');
    roller.inputNewData(temper.temperature,'t');
    roller.baroIndex++;
    roller.baroIndex%=ROLLING_AVG_LEN;
    delay(50);
  }
  originalTemper = roller.recieveData('t');
  originalBaro = roller.recieveData('b'); 
  
  srv.attach(SERVO_ONE); //closest to board
 // delay(3000);
  srvPos = 117;
  delay(3000);
}

void loop() {
  deltaT = lastT-millis()*1000;// in seconds
  lastT = millis()*1000;// in seconds
    sensors_event_t temper;
    sensors_event_t pressure;
    baro.getEvent(&pressure, &temper);
    roller.inputNewData(pressure.pressure,'b');
    roller.inputNewData(temper.temperature,'t');
    roller.baroIndex++;
    roller.baroIndex%=ROLLING_AVG_LEN;
    
    altitude[0] = pressToAlt(roller.recieveData('b'));
    altitude[1] = altitude[0];
    altitudeV[0] = (altitude[1]-altitude[0])/deltaT;
    altitudeV[1] = altitude[0];
    altitudeA = (altitudeV[1]-altitudeV[0])/deltaT;//bruh second degree derivations go crazy

  switch (state){
  
  case 0://on launch pad

    if (altitude[0]> 30){ 
      state = 1;
      Serial.println("launch detected, beginning logging");
      pinMode(LED_BUILTIN,OUTPUT);
      digitalWrite(LED_BUILTIN,HIGH);
    }

    break;

  // put your main code here, to run repeatedly:
  
  case 1://accelerating
    if (consecMeasurements >= 3){//exit loop for when the rocket is at appogee
        state = 2;
        consecMeasurements=0;
        Serial.println("drag flap deploy");
      }
    else if (altitude[0]>120){
        consecMeasurements++;
      }
    else{
        consecMeasurements = 0;
      }
    writeSDData();
    break;

    
  case 2:
    srvPos = 10;
    if (consecMeasurements == 3){//exit loop for when the rocket is at appogee
        state = 2;
        consecMeasurements=0;
        Serial.println("apogee reached");
      }
    else if (altitudeV[0]<0){
        consecMeasurements++;
      }
    else{
        consecMeasurements = 0;
      }
    writeSDData();
    break;

  
  case 3://freefall
    writeSDData();
    if (consecMeasurements == 3){//exit loop for when the rocket is at appogee
        state = 2;
        consecMeasurements=0;
        Serial.println("touched ground");
      }
    else if (altitudeV[0] < -0.2){
        consecMeasurements++;
    }
    else{
        consecMeasurements = 0;
      }
    break;
  }

  
  //Serial.println((String)roller.recieveRawData('b')+' ');
  //Serial.print((String)altitude[0] +' ');
  //Serial.print((String)altitudeV[0]);
  srv.write((srvPos-srvOffsets));
}


/**
float radiansToDegrees(float angle){
  return angle*180/PI;
}
//Drag Flaps Functions:
float getPastK(float accel,float v){//acceleration without gravity
  return ((accel+9.8)/pow(v,2));//check equation pls
}
float predictApogee (float k,float v){ // finds the distance to max alt, takes v and k - ONLY VALID FOR COASTING
//returns the distance from now to the final predicted apogee
return log((v*v*k/9.81)+1)/(2.0*k);
} //https://www.rocketmime.com/rockets/qref.html for range equation
float inverseApogee(float desiredApogee, float v) { // Working & Tested
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
int linreg(int n, volatile float x[], volatile float y[], volatile float* m, volatile float* b, float* r){ //stolen code
  float sumx = 0;                      // sum of x     
  float sumx2 = 0;                     // sum of x**2  
  float sumxy = 0;                     // sum of x * y 
  float sumy = 0;                      // sum of y    
  float sumy2 = 0;                     // sum of y**2  
  for (int i=0;i<n;i++){ 
    sumx  += x[i];       
    sumx2 += sqrt(x[i]);  
    sumxy += x[i] * y[i];
    sumy  += y[i];      
    sumy2 += sqrt(y[i]); 
  }

  float denom = (n * sumx2 - sqrt(sumx));
  if (denom == 0) {//vertical line 
      // singular matrix. can't solve the problem.
      // *m = 0;
      // *b = 0;
    if (r) *r = 0;
      return 1;
  }

  *m = (n * sumxy  -  sumx * sumy) / denom;
  *b = (sumy * sumx2  -  sumx * sumxy) / denom;
  if (r!=NULL) {
      *r = (sumxy - sumx * sumy / n) /    //compute correlation coeff 
            sqrt((sumx2 - sqrt(sumx)/n) *
            (sumy2 - sqrt(sumy)/n));
  }
  return 0; 
}
float getDesiredFlapAngle(volatile float m, volatile float b, float desiredk){//gets servo best angle from m, b, and desired K
  return (asin(sqrt(desiredk - b)/(m)));//in radians
}
float getAngleFactor(float theta){//in radians
  return (pow(sin(theta),2));//allows linear regression to work for angles linearly
}
float finalcalculation (){ // what tf u think this does, it predicts k using current state using acceleration data and current flap angle
//lin regress - least square method
if(!linreg(ROLLING_AVG_LEN,flapAngleAndKTable[0],flapAngleAndKTable[1],&m,&b,&correl)){ //if linear regression is succesfull (not vertical line)
  float ktarget = inverseApogee(TARGET_HEIGHT - (altitude[0]+altitude[1])/2.0, (altitudeV[0]+altitudeV[1])/2);
  return getDesiredFlapAngle(m,b,ktarget);
} else {
  Serial.println("lin reg failed");
  float ktarget = inverseApogee(TARGET_HEIGHT - (altitude[0]+altitude[1])/2.0, (altitudeV[0]+altitudeV[1])/2.0); //still hold current method because linear regression wont change m or b
  return getDesiredFlapAngle(m,b,ktarget); //may be problematic - needs testing
}
}

**/