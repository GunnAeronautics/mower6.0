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
  int state=0;

  unsigned long lastBeepTime = 0; //the last time the beep happened during case 4 (beep)

  volatile float srvPos[5]; //servo position array
  float srvOffsets[5] = {0,0,0,0,0};

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
float velocityZbaro[2];
float velocityZAccel[2];
float temperature;
volatile float rocketAngle[3];//integrated
volatile float desiredAngle;

  //Objects
File dataFile;
  //Debouncing for switches
Bounce sw1,sw2;

Adafruit_LSM6DS imu;
Adafruit_LPS22 baro;

Servo srv[5];

roll roller;
class roll{//tested
  public:
    float acclRaw[3][ROLLING_AVG_LEN]; //x,y,z
    float gyroRaw[3][ROLLING_AVG_LEN]; //x,y,z
    float baroRaw[ROLLING_AVG_LEN];
    float baroTempRaw[ROLLING_AVG_LEN];
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
          if (baroRaw[i]>=max){
            baroRaw[i]= ceiling;
          } else if (baroRaw[i]<=min){
            baroRaw[i]=floor;
          }
        }
      
      break;
      
      default:
      break;

    }
   }

    void shiftArray(float newData, float *array){
      for (int i=0; i<ROLLING_AVG_LEN-1; i++){ //downshift all values
         array[i]=array[i+1];
      }
      array[ROLLING_AVG_LEN-1]=newData; //replace final value with the new data
      return;
    }

    float getAvgInRollingAvg(float array[]){
      float sum = 0;
      return ((std::accumulate(array,array+ROLLING_AVG_LEN,sum))/ROLLING_AVG_LEN);
    }

    void inputNewData(float newdata, char datatype){
      switch (datatype) { 
        case 'X': shiftArray(newdata,acclRaw[0]) ; break;
        case 'Y': shiftArray(newdata,acclRaw[1]) ; break;
        case 'Z': shiftArray(newdata,acclRaw[2]) ; break;
        case 'x': shiftArray(newdata,gyroRaw[0]) ; break;
        case 'y': shiftArray(newdata,gyroRaw[1]) ; break;
        case 'z': shiftArray(newdata,gyroRaw[2]) ; break;
        case 'b': shiftArray(newdata,baroRaw)    ; break;
        case 't': shiftArray(newdata,baroTempRaw); break;
      }
      
    }



    float recieveNewData(char datatype){
      switch (datatype) {
        case 'X': return getAvgInRollingAvg(acclRaw[0]) ; break;
        case 'Y': return getAvgInRollingAvg(acclRaw[1]) ; break;
        case 'Z': return getAvgInRollingAvg(acclRaw[2]) ; break;
        case 'x': return getAvgInRollingAvg(gyroRaw[0]) ; break;
        case 'y': return getAvgInRollingAvg(gyroRaw[1]) ; break;
        case 'z': return getAvgInRollingAvg(gyroRaw[2]) ; break;
        case 'b': return getAvgInRollingAvg(baroRaw) ; break;
        case 't': return getAvgInRollingAvg(baroTempRaw) ; break;
      }
      return 0;
    }

    float recieveRawData(char datatype){
      switch (datatype) {
        case 'X': return acclRaw[0][ROLLING_AVG_LEN-1] ; break;
        case 'Y': return acclRaw[1][ROLLING_AVG_LEN-1] ; break;
        case 'Z': return acclRaw[2][ROLLING_AVG_LEN-1] ; break;
        case 'x': return gyroRaw[0][ROLLING_AVG_LEN-1] ; break;
        case 'y': return gyroRaw[1][ROLLING_AVG_LEN-1] ; break;
        case 'z': return gyroRaw[2][ROLLING_AVG_LEN-1] ; break;
        case 'b': return baroRaw[ROLLING_AVG_LEN-1] ; break;
        case 't': return baroTempRaw[ROLLING_AVG_LEN-1] ; break;
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
        
  fname="mower"+(String)0+".csv";

  for (int i=0; i<999&&(SD.exists(fname));i++){ //add detection if file already exists
    fname="mower"+(String)i+".csv";
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


  for (int i=0; (i < 5); i++){// initialize reference ground measurements to find altitude change
    getBaroDat();// during flight
    delay(5);
  }
  referenceGroundPressure = roller.recieveNewData('b');//in pascals
  referenceGroundTemperature = roller.receiveNewData('t');// in celsius

  //analog peripherals

//attatching switches to debouncers
  sw1.attach(CTRL_SW1,INPUT); //attaching debouncer to switches
  sw2.attach(CTRL_SW2,INPUT); 
//Servos
  srv[0].attach(SRV1_PIN);
  srv[1].attach(SRV2_PIN);
  srv[2].attach(SRV3_PIN);
  srv[3].attach(SRV4_PIN);
  srv[4].attach(SRV5_PIN);


  buzztone(1,1000); //buzz for peripheral initialization done
delay(9000);
//Check if test is active (SW1 switched HIGH)
state = 1; //arm rocket
}

void setup1(){ //core 2 setup function
while(state==0){ //
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
      #ifdef ISCANARD //canards control loop




      #endif

      #ifdef ISDRAGFLAP //drag flap control loop
        //add current K value to array, a=n*v^, n=k/m -> k=ma/v^2 (gravity not measured -> az = Fd)
        for(int i=0; i<ROLLING_AVG_LEN-1;i++){
          flapAngleKData[0][i]=flapAngleKData[0][i+1];
          flapAngleKData[0][i]=flapAngleKData[0][i+1];
        }
        flapAngleKData[1][ROLLING_AVG_LEN-1]=roller.acclRaw[2]*(MASS/(velocityZbaro[1]*velocityZbaro[1]));
      
      #endif



        /*TRANSITION*/
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
      srv[5].write(30); //swag money

    writeSDData();
    break;


    case 5: //landed - just beep periodically
      if (millis() > lastBeepTime + 2000) { //FIX MAX MILLIS CASE
        buzztone(50); // TODO we dont know if buzztone takes time in seconds or in milliseconds
        lastBeepTime = millis();
      }
    break;
  }
    #ifdef ISCANARD
    if ((abs(srvPos[i])>SRV_MAX_ANGLE)&&i<5){ //clop srv angle if canarding and not the dual deployment servo
      srvPos[i]=(srvPos[i]/abs(srvPos[i]))*SRV_MAX_ANGLE; 
    }
    #endif
    for (int i=0; i<5;i++){ //edit once final number of servos is confirmed
srv[i].write((srvPos[i]-srvOffsets[i]));
    }
    
  getIMUDat();
  getBaroDat();
  }





float sensorDeltaT;
volatile long prevSensorMillis;
float altitude[2];

void baroAccelDataFilter() {
  //constants for weighted average
  float baroConstant = 0.5;
  float accelConstant = 0.5;
  float accl = roller.receiveNewData('Z') * sin(pitchAngleFiltered);
  float baro = roller.receiveNewData('z');
  float estimate = (baroConstant * baro) + (accelConstant * accl);
  velocityZ += sensorDeltaT*estimate;
}

void loop1(){ //Core 2 loop - does data filtering when data is available
  // does heavy calculations because calculations are heavy
  /*TODO:
  add way to turn raw variables into "actual form"
  take gravity impact into account on integration
  */
  if(state==2 || state==3 || state==4){ //needed

    if(newSensorDat){

    
    sensorDeltaT=millis()-prevSensorMillis;
    if(sensorDeltaT<0){
      
    }
    prevSensorMillis=millis();
   
    newSensorDat=false;
  }
  if (state==2 && newSensorDat){

  // does heavy calculations because calculations are heavy
  //FORMAT NEEDS CHANGE
  if(state==2 || state==3 || state==4){


      sensorDeltaT = (millis()-prevSensorMillis)/1000;
      
      velocityX+=roller.recieveNewData('X')*sensorDeltaT;//bad integration - needs clipping (? - measure impact at some point)
      velocityY+=roller.recieveNewData('Y')*sensorDeltaT;
      velocityZ+=roller.recieveNewData('Z')*sensorDeltaT; 

      rocketAngle[0]+=roller.recieveNewData('x')*sensorDeltaT;
      rocketAngle[1]+=roller.recieveNewData('y')*sensorDeltaT;
      rocketAngle[2]+=roller.recieveNewData('z')*sensorDeltaT;
      
    
    
      //do filtering to get pitch angles
          sensorDeltaT = (millis() - prevSensorMillis)/1000;
          altitude[0]=altitude[1];
          altitude[1] = pressureToAlt(roller.recieveNewData('b'));
          velocityZbaro[0]=velocityZbaro[1];//math wizardry VVV
          velocityZbaro[1]=(altitude[1]-altitude[0])/sensorDeltaT;

          #ifdef ISDRAGFLAP
          nFinder();
          #endif

          #ifdef ISCANARD //pitch angles only needed when doing canards
           float vMagAccl =sqrt((float)(velocityX*velocityX)+(velocityY*velocityX)+(velocityZ*velocityZ));//rocket total velocity
          float pitchEstimateAcclBaro = asin(velocityZbaro[1]/vMagAccl);
          rollangle = (pid(integral[0], previousError[0], rocketAngle[0], 0, sensorDeltaT, ROLLKp, ROLLKi, ROLLKd))*SERVOCOEFFICIENT;
          srvPos[0] = pid(integral[1], previousError[1], rocketAngle[1], desiredAngle, sensorDeltaT, PITCHKp, PITCHKp, PITCHKp)*SERVOCOEFFICIENT;
          srvPos[1] = rollangle+pid(integral[2], previousError[2], rocketAngle[2], 0, sensorDeltaT, YAWKp, YAWKi, YAWKd)*SERVOCOEFFICIENT;      
          srvPos[2] = -srvPos[0];
          srvPos[3] = rollangle-srvPos[1];
          #endif
          prevSensorMillis = millis();
          newSensorDat = false;
          //update desired angle using this equation
    }
}
  }
}


//Sensor interrupt functions, TODO: change them per state (no need for data filtering if on the way down)


void getIMUDat(){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t tempIMU;
  imu.getEvent(&accel,&gyro,&tempIMU);
  newSensorDat=true;
    
  roller.inputNewData(accel.acceleration.x, 'X');
  roller.inputNewData(accel.acceleration.y, 'Y');
  roller.inputNewData(accel.acceleration.z, 'Z'); 

  roller.inputNewData(gyro.gyro.x, 'x');
  roller.inputNewData(gyro.gyro.y, 'y');
  roller.inputNewData(gyro.gyro.z, 'z');
  
  } 


void getBaroDat(){ //when barometric pressure data is available
  newSensorDat=true;
  sensors_event_t pressure;
  sensors_event_t tempBaro;
    

  baro.getEvent(&pressure,&tempBaro);//hecta pascals

  roller.inputNewData(pressure.pressure, 'b');
  roller.inputNewData(tempBaro.temperature, 't');
} 

//Perephrial functions
unsigned long srvSweepTime;
void srvSweep(){ //sweeps all servoes between 0 degrees and SRV_MAX_POS every SRV_SWEEP_TIME without any delay functions
  srvSweepTime=millis();
  float srvPosAfterSweep = ((initialSweepMillis-srvSweepTime)%SRV_SWEEP_TIME)*90; //modulo makes it wrap back around
  for (int i=0; i<4; i++){
    srvPos[i]=srvPosAfterSweep;
  }
}

void buzztone (int time,int frequency = 1000) { //default frequency = 1000 Hz
  tone(BUZZ_PIN,frequency,time);
}

String dataString; //s
void writeSDData (){

  dataString =(String)millis()+','+
              (String)(int)roller.recieveRawData('X')*100+','+
              (String)(int)roller.recieveRawData('Y')*100+','+
              (String)(int)roller.recieveRawData('Z')*100+','+
              (String)(int)roller.recieveRawData('x')*100+','+
              (String)(int)roller.recieveRawData('y')*100+','+
              (String)(int)roller.recieveRawData('z')*100+','+
              (String)(int)roller.recieveRawData('b')*100+','+
              char(state);

    File dataFile = SD.open(fname, FILE_WRITE);
  dataFile.println(dataString);
  dataFile.close();
}



//Helper functions 

float pressureToAlt(float pres){ //returns alt (m) from pressure in hecta pascals and temperature in celsius
  return (float)(((273+referenceGroundTemperature)/(-.0065))*((pow((pres/referenceGroundPressure),((8.314*.0065)/(9.807*.02896))))-1)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}



unsigned long predictLandTime() {
  //TODO
  //baroAltitude[0] - REF_GROUND_ALTITUDE + velocityZ*t + 0.5*acclZ*t^2
  //t = (-velocityZ*t + sqrt(velocityZ^2-4(height)(0.5*acclZ))/2*height
  float height = roller.inputNewData('h');
  float a = 0.5 * roller.recieveData('z');
}

float pid(float *integral, float *previousError, float currentState, float desiredState, float deltaT, float Kp, float Ki, float Kd){//modular PID state transfer function
  float error = desiredState-currentState;

  float P = Kp * error;//porportional

  *integral += error;

  float I = Ki * (*integral);//integral

  float derivative = (error-*previousError) / deltaT;
  float D = Kd * derivative;//derivative
  float output = P+I+D;

  if (output > SRV_MAX_ANGLE){//failsafe
    output = SRV_MAX_ANGLE;
  }else if( output < (-SRV_MAX_ANGLE)){
    output = (-SRV_MAX_ANGLE);
  }
  *previousError = error;

  return output;
}

unsigned long predictLandTime() {
  float height = altitude[1];
  float a = 0.5 * roller.acclZ[0][ROLLING_AVG_LEN-1];

  float b = velocityZ * velocityZ;
  float c = height;
  return (-b + sqrt(b*b-(4*a*c)))/2; //quadratic formula
}

#ifdef ISCANARD //canard speciifc helpers and variables

#define ROLLKp .1
#define ROLLKi .1
#define ROLLKd .1
#define PITCHKp .1
#define PITCHKi .1
#define PITCHKd .1
#define YAWKp .1
#define YAWKi .1
#define YAWKd .1

#define SERVOCOEFFICIENT .01 //output of pid is multiplied by this to get the servo angle

#define PITCHFUNCTION (100/(1+pow(2.7,-((altitude[1]-200)/25)))-1.8)

float integral[3];
float previousError[3];
float rollangle;



#endif


#ifdef ISDRAGFLAP //drag flap specific helpers and variables

static int MASS= .62; //MASS WITHOUT PROPELLANT - CALCULATE AT LAUNCH

volatile float currB; //base value of k from linear regression
volatile float currM; //how much k varies by sin pitch angle
volatile float dragFAmgle;
volatile float currN; //n=k/mass
float correl;
float flapAngleKData[2][ROLLING_AVG_LEN];
#define CURRK currN*MASS
#define INVERSE_APOGEE_TOLERANCE .5 //in meters, + or -

float getPastK(float mass,float accel,float v){
  float k = ((mass*accel+9.8)/pow(v,2));
  return k;
}

//PID 
float predictApogee (float k,float v){ // finds the distance to max alt, takes v and k - ONLY VALID FOR COASTING
float h = log((v*v*k/9.81)+1)/(2*k);//returns the distance from now to the final predicted apogee
return k;
} //https://www.rocketmime.com/rockets/qref.html for range equation
 
float inverseApogee(float desiredApogee) { // returns a k given desired apogee, tested, NO BOUNDS PROTECTION
  float currPrediction, currStepSize;
  float currX = .001;
  bool found = false;
  currStepSize = .5;
  for (int n = 0; n<40; n++) {
    // chose if we want to use velocityz or velocityzbaro

    currPrediction = predictApogee(velocityZ, currX + currStepSize);
    if (currPrediction < (desiredApogee + INVERSE_APOGEE_TOLERANCE) &&
        currPrediction > (desiredApogee - INVERSE_APOGEE_TOLERANCE)) { 
      currX += currStepSize;
      return currX;
    } else if (currPrediction < desiredApogee) {
      currStepSize /= 2; // reduce prediction step size
    } else if (currPrediction > desiredApogee) {
      currX += currStepSize;
    }
  }
  return currX;
}

int linreg(int n, const float x[], const float y[], volatile float* m, volatile float* b, float* r){ //stolen code
    float   sumx = 0.0;                      /* sum of x     */
    float   sumx2 = 0.0;                     /* sum of x**2  */
    float   sumxy = 0.0;                     /* sum of x * y */
    float   sumy = 0.0;                      /* sum of y     */
    float   sumy2 = 0.0;                     /* sum of y**2  */

    for (int i=0;i<n;i++){ 
        sumx  += x[i];       
        sumx2 += sqrt(x[i]);  
        sumxy += x[i] * y[i];
        sumy  += y[i];      
        sumy2 += sqrt(y[i]); 
    } 

    float denom = (n * sumx2 - sqrt(sumx));
    if (denom == 0) {
        // singular matrix. can't solve the problem.
        *m = 0;
        *b = 0;
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


bool nFinder (){ // what tf u think this does, it predicts k using current state using acceleration data and current flap angle
//lin regress - least square method
if(linreg(ROLLING_AVG_LEN,flapAngleKData[0],flapAngleKData[1],&currM,&currB,&correl)){ //check again
  currN=sin(pitchAngleFiltered)*currM+currB;
  return 1;
} else {
  Serial.println("lin reg failed");
  return 0;
}
} 
float getFlapAngle(){ //finds the FLAP ANGLE - NOT THE SERVO ANGLE - uses PID to go towards the desired apogee
if(nFinder){
   
}
}


#endif

