#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Bounce2.h>
#include <Servo.h>
//using adafruit's libraries
#include <Adafruit_LSM6DS.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <numeric>
#include <math.h>//for log
/*


the LPS22 is reffered to as baro here, not altimeter because shorter, LSM6DSMXX is reffered to as IMU

SPI frequency currently set by setclockdivider(16) to around 8 MHz

*/


//#define SPI_FREQ 10000000 
 // IMPORTANT: ADJUST BEFOER FLIGHT
#define TARGET_TIME 44.5 //in seconds
#define TARGET_HEIGHT 250//in meters





//debug mode adds serial messages and some extra stuff
//IMPORTANT: IF YOU WANT TO DISABLE, COMMENT OUT, DONT SET FALSE
#define ISDEBUG true
//#define ISCANARD true
#define ISDRAGFLAP true


/* template for debug message:

#ifdef ISDEBUG
Serial.println(" ");
#endif

*/
#define ACCEL_THRESH 18 
#define ALT_THRESH 25 //meters above initial
#define SRV_MAX_ANGLE 7 //in degrees
#define ROCKET_ANGLE_TOLERANCE 5 //in degrees

#define IMU_DATA_RATE LSM6DS_RATE_52_HZ
#define BAROMETER_DATA_RATE LPS22_RATE_25_HZ

//amount of rolling average numbers to keep track of
#define ROLLING_AVG_LEN 5

#define SRV_SWEEP_TIME 2500//in millis

//Pin defs
#define SRV1_PIN 4
#define SRV2_PIN 5
#define SRV3_PIN 6
#define SRV4_PIN 7
#define SRV5_PIN 8
#define IMU_INT1 11
#define IMU_INT2 10
#define BARO_INT 12
#define DEBUG_LED 13
#define IMU_CS 9
#define SD_CS 14
//TX = DO = MOSI, RX = DI=MISO
#define SPI_SCLK 18
#define SPI_TX 19 //AKA mosi
#define SPI_RX 20 //AKA miso
#define BARO_CS 15
#define CTRL_SW1 26
#define CTRL_SW2 22
#define BUZZ_PIN 27 //using tone function


    //Runtime variables
int state=1;
unsigned long lastBeepTime = 0; //the last time the beep happened during case 4 (beep)
float srvPos; //servo position  not an array
float srvOffsets = 0;

volatile bool newSensorDat = false;

float referenceGroundPressure;
float referenceGroundTemperature;
int8_t consecMeasurements = 0; //this variable should never be greater than 4. Defined as 8-bit integer to save memory
unsigned long initialSweepMillis = 0;
//stuff for adafruit's libraries
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t pressure;
sensors_event_t tempIMU;
sensors_event_t tempBaro;

  //filtered data - keeping past 2 values to enable differentiation
float pitchAngleFiltered;//Not Used
float velocityX,velocityY,velocityZ;

volatile float velocityZbaro[2];
volatile float accelZBaro[2];
//float velocityZAccel[2];
float temperature;
volatile float rocketAngle[3];//integrated
volatile float desiredFlapAngle;
volatile float currentFlapAngle;
  //Objects
File dataFile;
  //Debouncing for switches
Bounce sw1,sw2;

Adafruit_LSM6DS imu;
Adafruit_LPS22 baro;

Servo srv;

roll roller;
class roll { // tested
public:
  float acclRaw[3][ROLLING_AVG_LEN]; // x,y,z
  float gyroRaw[3][ROLLING_AVG_LEN]; // x,y,z
  float baroRaw[ROLLING_AVG_LEN];
  float baroTempRaw[ROLLING_AVG_LEN];

  int imuIndex = 0;
  int baroIndex = 0;
  ;
  void clipVariables(float max, float ceiling, float min, float floor,
                     int arrayToUse) { // case 1 = accl, 2= gyro, 3= altitude
    switch (arrayToUse) {
    case (1): // accel data
      for (int j = 0; j < 3; j++) {
        for (int i = 0; i < ROLLING_AVG_LEN; i++) {
          if (acclRaw[j][i] >= max) {
            acclRaw[j][i] = ceiling;
          } else if (acclRaw[j][i] <= min) {
            acclRaw[j][i] = floor;
          }
        }
      }

      break;

    case (2): // gyro data
      for (int j = 0; j < 3; j++) {
        for (int i = 0; i < ROLLING_AVG_LEN; i++) {
          if (gyroRaw[j][i] >= max) {
            gyroRaw[j][i] = ceiling;
          } else if (gyroRaw[j][i] <= min) {
            gyroRaw[j][i] = floor;
          }
        }
      }
      break;

    case (3): // alt data

      for (int i = 0; i < ROLLING_AVG_LEN; i++) {
        if (baroRaw[i] >= max) {
          baroRaw[i] = ceiling;
        } else if (baroRaw[i] <= min) {
          baroRaw[i] = floor;
        }
      }

      break;

    default:
      break;
    }
  }

  void shiftArray(float newData, float *array, int index) {
    array[index] = newData; // replace index value with the new data
    return;
  }

  float getAvgInRollingAvg(float array[]) {
    float sum = 0;
    return ((std::accumulate(array, array + ROLLING_AVG_LEN, sum)) /
            ROLLING_AVG_LEN);
  }

  void inputNewData(float newdata, char datatype) {
    switch (datatype) {
    case 'X':shiftArray(newdata, acclRaw[0], imuIndex);break;
    case 'Y':shiftArray(newdata, acclRaw[1], imuIndex);break;
    case 'Z':shiftArray(newdata, acclRaw[2], imuIndex);break;
    case 'x':shiftArray(newdata, gyroRaw[0], imuIndex);break;
    case 'y':shiftArray(newdata, gyroRaw[1], imuIndex);break;
    case 'z':shiftArray(newdata, gyroRaw[2], imuIndex);break;
    case 'b':shiftArray(newdata, baroRaw, baroIndex);break;
    case 't':shiftArray(newdata, baroTempRaw, baroIndex);break;
    }
  }

  float recieveNewData(char datatype) {
    switch (datatype) {
    case 'X':return getAvgInRollingAvg(acclRaw[0]);break;
    case 'Y':return getAvgInRollingAvg(acclRaw[1]);break;
    case 'Z':return getAvgInRollingAvg(acclRaw[2]);break;
    case 'x':return getAvgInRollingAvg(gyroRaw[0]);break;
    case 'y':return getAvgInRollingAvg(gyroRaw[1]);break;
    case 'z':return getAvgInRollingAvg(gyroRaw[2]);break;
    case 'b':return getAvgInRollingAvg(baroRaw);break;
    case 't':return getAvgInRollingAvg(baroTempRaw);break;
    }
    return 0;
  }

  float recieveRawData(char datatype) {
    switch (datatype) {
    case 'X':return acclRaw[0][imuIndex];break;
    case 'Y':return acclRaw[1][imuIndex];break;
    case 'Z':return acclRaw[2][imuIndex];break;
    case 'x':return gyroRaw[0][imuIndex];break;
    case 'y':return gyroRaw[1][imuIndex];break;
    case 'z':return gyroRaw[2][imuIndex];break;
    case 'b':return baroRaw[baroIndex];break;
    case 't':return baroTempRaw[baroIndex];break;
    }
    return 0;
  }
};

String fname;

void setup() {

//communication interface begins
  Serial.begin(115200);

  delay(6000);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN,LOW);
  //SPI

  SPI.setRX(20); //core already manages which spi to use (using SPI0)
  SPI.setTX(19);
  SPI.setSCK(18);

    //sensors
      //IMU

  if (!imu.begin_SPI(IMU_CS)){
    Serial.println("IMU did not initialize");
    while(true);
  }

        //config imu speed
  imu.setGyroDataRate(IMU_DATA_RATE);
  imu.setAccelDataRate(IMU_DATA_RATE);
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS); //should be sufficient


      //baro
  baro.setDataRate(BAROMETER_DATA_RATE);
  if (!baro.begin_SPI(BARO_CS)){
    Serial.println("Baro did not initialize");
    while(true);
  }

        //config baro speed

      //SD card
  if(!SD.begin(SD_CS)){
    Serial.println("SD did not initialize");
    while(true);
  }
        
  fname="lawn"+(String)0+".csv";
  for (int i=0; i<999&&(SD.exists(fname));i++){ //add detection if file already exists
    fname="lawn"+(String)i+".csv";
    Serial.print("tried ");
    Serial.println(fname);
  }
  File dataFile = SD.open(fname, FILE_WRITE);
  
  if (!dataFile){
    Serial.println("dat file not initialized, name = ");
    Serial.print(fname);
    while (true){
      delay(50);
    }
  } else {
    Serial.println("dat file successfully initialized, name = ");
    Serial.print(fname);
  }
        //begin file with header
  dataFile.println("time, x accl, y accl, z accl, gyro x, gyro y, gyro z, pressure");
  dataFile.close();


  for (int i=0; (i < ROLLING_AVG_LEN); i++){// initialize reference ground measurements to find altitude change
    getBaroDat();// during flight
    delay(50);
  }
  referenceGroundPressure = roller.recieveNewData('b');//in pascals
  referenceGroundTemperature = roller.receiveNewData('t');// in celsius

  //analog peripherals

//attatching switches to debouncers
  sw1.attach(CTRL_SW1,INPUT); //attaching debouncer to switches
  sw2.attach(CTRL_SW2,INPUT); 
//Servos
  srv.attach(SRV1_PIN);



  buzztone(1,1000); //buzz for peripheral initialization done
delay(9000);
//Check if test is active (SW1 switched HIGH)
state = 1; //arm rocket
}

void setup1(){ //core 2 setup function
while(state==0){ //syncs both cores to start their loops together
  delayMicroseconds(10);
}
}



void loop() { //Loop 0 - does control loop stuff


  getIMUDat();
  getBaroDat();
  prevSensorMillis = millis();


  switch (state)
  {
    case 0: //test
      delay(100);
      Serial.println(roller.recieve('X')+ ' ');
      Serial.print(roller.recieve('Y')+ ' ');
      Serial.print(roller.recieve('Z'));
      Serial.println(roller.recieve('x')+ ' ');
      Serial.print(roller.recieve('y'+ ' '));
      Serial.print(roller.recieve('z'));
      Serial.println(roller.recieve('b'));
    break;

    case 1://on pad - waiting for high acceleration, major change in pressure TODO: maybe configure one of the interrupts on imu for wakeup signal, (or 6d interrupt)
      if (consecMeasurements == 3){//exit loop for when the rocket is at appogee
        state = 2;
        consecMeasurements=0;
      }
      else if ((roller.recieveNewData('Y')>ACCEL_THRESH)||(pressureToAlt(roller.recieveNewData('b'))>ALT_THRESH)){
        consecMeasurements++;
      }
      else{
        consecMeasurements = 0;
      }
      break;  

    case 2:
      if (consecMeasurements == 3){//exit loop for when the rocket above 100 meters
        state = 3;
        consecMeasurements=0;
      }
      else if ((pressureToAlt(roller.recieveNewData('b'))>100)){// if rocket over 100 meters up then do thing
        consecMeasurements++;
      }
      else{
        consecMeasurements = 0;
      }
      writeSDData();
      break;

    case 3: //on way up - doing everything (turning n stuff) 
      
        /*TRANSITION*/
      srvPos = desiredFlapAngle;//hypothetical 1-1 ratio for flap angle to servo angle
      currentFlapAngle = desiredFlapAngle;

      if (consecMeasurements == 3){//exit loop for when the rocket is at appogee also haha 420
        state = 3;
        consecMeasurements=0;
      }
      else if (velocityZbaro < 0){
        consecMeasurements++;
      }
      else{
        consecMeasurements = 0;
      }
      writeSDData();
      break;

    case 4: //after apogee - when altitude decreases for 15 consecutive measurements - just need to worry about chute deployment
      if (TARGET_TIME > predictLandTime()) { //if the target time is less than the predicted landing time, nothing needs to be done
        consecMeasurements = 0; //if this condition passes then consecMeasurements should be zero (consec measurement streak lost or never started)
        break;
      }

      //if the previous condition passed, we must check consecMeasurements to determine whether we should deploy the chute
      if (consecMeasurements < 4) { //if target
        consecMeasurements++; 
        break;
      }

      //RELEASE CHUTE
      //do something to servo 5
      srv.write(30); //swag money

    writeSDData();
    break;


    case 5: //landed - just beep periodically
      if (millis() > lastBeepTime + 2000) { //FIX MAX MILLIS CASE
        buzztone(50); // TODO we dont know if buzztone takes time in seconds or in milliseconds
        lastBeepTime = millis();
      }
    break;
  }
  //edit once final number of servos is confirmed
  srv.write((srvPos-srvOffsets));
  
    
  getIMUDat();
  getBaroDat();
  newSensorDat=true;
  }





float sensorDeltaT;
volatile long prevSensorMillis;
volatile float altitude[2];
int angleKTableIndex;
void loop1(){ //Core 2 loop - does data filtering when data is available
  // does heavy calculations because calculations are heavy
  /*TODO:
  add way to turn raw variables into "actual form"
  take gravity impact into account on integration
  */
 /*
  if(state==2 || state==3 || state==4){ //needed

    if(newSensorDat){

    
    sensorDeltaT=millis()-prevSensorMillis;
    if(sensorDeltaT<0){
      
    }
    prevSensorMillis=millis();
   
    //newSensorDat=false;
  }
  */
  if (state==3 && newSensorDat){//75 hz max

  // does heavy calculations because calculations are heavy
    sensorDeltaT = (millis()-prevSensorMillis)/1000;
    
    velocityX+=roller.recieveNewData('X')*sensorDeltaT;//bad integration - needs clipping (? - measure impact at some point)
    velocityY+=roller.recieveNewData('Y')*sensorDeltaT;
    velocityZ+=roller.recieveNewData('Z')*sensorDeltaT; 
    rocketAngle[0]+=roller.recieveNewData('x')*sensorDeltaT;
    rocketAngle[1]+=roller.recieveNewData('y')*sensorDeltaT;
    rocketAngle[2]+=roller.recieveNewData('z')*sensorDeltaT;
    //do filtering to get pitch angles
        
    
    altitude[0]=altitude[1];//displacement
    altitude[1] = pressureToAlt(roller.recieveNewData('b'));
    
    velocityZbaro[0]=velocityZbaro[1];//derivative
    velocityZbaro[1]=(altitude[1]-altitude[0])/sensorDeltaT;

    accelZBaro[0] = accelZBaro[1];//derivative
    accelZBaro[1] = (velocityZbaro[1] - velocityZbaro[0])/sensorDeltaT;

    //add current K value to array, a=n*v^, n=k/m -> k=ma/v^2 (gravity not measured -> az = Fd)
    flapAngleKData[0][angleKTableIndex] = getAngleFactor(currentFlapAngle); //ANGLE OF DRAG FLAPS!!!
    flapAngleKData[1][angleKTableIndex] = getPastK(accelZBaro[1], (velocityZbaro[1])); //CURRENT K VALUE

    desiredFlapAngle = finalcalculation();

    prevSensorMillis = millis();
    newSensorDat = false;
        //update desired angle using this equation
  }
}




//Sensor interrupt functions, TODO: change them per state (no need for data filtering if on the way down)


void getIMUDat(){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t tempIMU;
  imu.getEvent(&accel,&gyro,&tempIMU);
    
  roller.inputNewData(accel.acceleration.x, 'X');
  roller.inputNewData(accel.acceleration.y, 'Y');
  roller.inputNewData(accel.acceleration.z, 'Z'); 

  roller.inputNewData(gyro.gyro.x, 'x');
  roller.inputNewData(gyro.gyro.y, 'y');
  roller.inputNewData(gyro.gyro.z, 'z');
  roller.imuIndex++;
  roller.imuIndex%=ROLLING_AVG_LEN;
  } 

void getBaroDat(){ //when barometric pressure data is available
  sensors_event_t pressure;
  sensors_event_t tempBaro;
    

  baro.getEvent(&pressure,&tempBaro);//hecta pascals

  roller.inputNewData(pressure.pressure, 'b');
  roller.inputNewData(tempBaro.temperature, 't');
  roller.baroIndex++;
  roller.baroIndex%=ROLLING_AVG_LEN;
} 

//Data filtering functions

void baroAccelDataFilter() {//unused
  //constants for weighted average
  float baroConstant = 0.5;
  float accelConstant = 0.5;
  float accl = roller.receiveNewData('Z') * sin(pitchAngleFiltered);
  float baro = roller.receiveNewData('z');
  float estimate = (baroConstant * baro) + (accelConstant * accl);
  velocityZ += sensorDeltaT*estimate;
}

//Perephrial functions

float srvPosAfterSweep;
void srvSweep(){ //sweeps all servoes between 0 degrees and SRV_MAX_POS every SRV_SWEEP_TIME without any delay functions
  unsigned long srvSweepTime=millis();
  srvPosAfterSweep = ((initialSweepMillis-srvSweepTime)%SRV_SWEEP_TIME)*90; //modulo makes it wrap back around
  srvPos=srvPosAfterSweep;
}

void buzztone (int time,int frequency = 1000) { //default frequency = 1000 Hz
  tone(BUZZ_PIN,frequency,time);
}

void writeSDData (){

  String dataString =(String)millis()+','+
    //useful for analysis
    (String)roller.recieveRawData('X')+','+
    (String)roller.recieveRawData('Y')+','+
    (String)roller.recieveRawData('Z')+','+
    (String)roller.recieveRawData('x')+','+
    (String)roller.recieveRawData('y')+','+
    (String)roller.recieveRawData('z')+','+
    (String)roller.recieveRawData('b')+','+
    //useful during flights
    (String)altitude[0] + ',' + //current altitude
    (String)flapAngleKData[0][angleKTableIndex] + ',' + //current flap angle
    (String)flapAngleKData[1][angleKTableIndex] + ',' + //current k
    (String)desiredFlapAngle +',' +//final calculation desired flap angle (radians)
    (String)velocityZbaro[0] + ',' +
    char(state);//state of flight

    File dataFile = SD.open(fname, FILE_WRITE);
  dataFile.println(dataString);
  dataFile.close();
}

float radiansToDegrees(float angle){
  return angle*180/3.1415;
}

float pressureToAlt(float pres){ //returns alt (m) from pressure in hecta pascals and temperature in celsius
  return (float)(((273+referenceGroundTemperature)/(-.0065))*((pow((pres/referenceGroundPressure),((8.314*.0065)/(9.807*.02896))))-1)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}

unsigned long predictLandTime() {
  float height = altitude[1];
  float a = 0.5 * roller.acclZ[0][ROLLING_AVG_LEN-1];

  float b = velocityZ * velocityZ;
  float c = height;
  return (-b + sqrt(b*b-(4*a*c)))/2; //quadratic formula
}

 //drag flap specific helpers and variables

static float MASS= .62; //MASS WITHOUT PROPELLANT - CALCULATE AT LAUNCH

volatile float currB; //base value of k from linear regression
volatile float currM; //how much k varies by sin pitch angle
float correl;//unused
//volatile float dragFAmgle;
//volatile float currN; //n=k/mass

volatile float flapAngleKData[2][ROLLING_AVG_LEN];//flap angle converted to sin(theta)^2, k value
//#define CURRK currN*MASS
//#define INVERSE_APOGEE_TOLERANCE .5 //in meters, + or -

//Drag Flaps Functions:
float getPastK(float accel,float v){//acceleration without gravity
  return ((MASS*accel+9.8)/pow(v,2));//check equation pls
}

float predictApogee (float k,float v){ // finds the distance to max alt, takes v and k - ONLY VALID FOR COASTING
//returns the distance from now to the final predicted apogee
return log((v*v*k/9.81)+1)/(2*k);
} //https://www.rocketmime.com/rockets/qref.html for range equation
 

float inverseApogee(float desiredApogee, float v) { // Working & Tested
  float searchRangeH = 20;//true binary search (very accurate)
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
  float sumx = 0;                      /* sum of x     */
  float sumx2 = 0;                     /* sum of x**2  */
  float sumxy = 0;                     /* sum of x * y */
  float sumy = 0;                      /* sum of y     */
  float sumy2 = 0;                     /* sum of y**2  */
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
      *r = (sumxy - sumx * sumy / n) /    /* compute correlation coeff */
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
// float or integer output 
//lin regress - least square method
if(linreg(ROLLING_AVG_LEN,flapAngleKData[0],flapAngleKData[1],&currM,&currB,&correl)){ //check again
  float ktarget = inverseApogee(TARGET_HEIGHT - (altitude[0]+altitude[1])/2, (velocityZbaro[0]+velocityZbaro[1])/2);
  return getDesiredFlapAngle(currM,currB,ktarget);
} else {
  Serial.println("lin reg failed");
  float ktarget = inverseApogee(TARGET_HEIGHT - (altitude[0]+altitude[1])/2, (velocityZbaro[0]+velocityZbaro[1])/2);
  return getDesiredFlapAngle(currM,currB,ktarget);
}
}
//float getFlapAngle(){ //finds the FLAP ANGLE - NOT THE SERVO ANGLE - uses PID to go towards the desired apogee
//if(nFinder){ 
//}



