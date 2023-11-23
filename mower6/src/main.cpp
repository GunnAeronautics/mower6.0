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

//PID CONSTANTS
#define Kp 0.1//Present
#define Ki 0.1//Past
#define Kd 0.1//Future


//debug mode adds serial messages and some extra stuff
#define ISDEBUG true


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
  unsigned long prevMillis=0;
  unsigned long currT = 0;
  int loopTime = 0;
  unsigned long lastBeepTime = 0; //the last time the beep happened during case 4 (beep)

  volatile float srvPos[5]; //servo position array
  float srvOffsets[5] = {0,0,0,0,0};
  volatile bool newBaroDat = false;
  volatile bool newIMUDat = false;

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



//float angleFromIntegration[3];//depreciated until more data filtering is required

  //filtered data - keeping past 2 values to enable differentiation
float pitchAngleFiltered;//Not Used
float velocityX,velocityY,velocityZ;
float velocityZbaro[2];
float baroAltitude[2]; //keep the past 2 values
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
//rolling average
roll roller;//rolling object
class roll{//tested (it works)
  public:
    float acclRaw[3][ROLLING_AVG_LEN];
    float gyroRaw[3][ROLLING_AVG_LEN];
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



  //interrupt function for when imu data is ready
void imuDatRdy(); 
  //interrupt function for when barometer data is ready
void baroDatRdy(); 
  //Buzzes a tone at frequency in Hz and for time in ms (i think)
void buzztone (int frequency,int time);



void setup() {

//communication interface begins
  Serial.begin(115200);

  delay(6000);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN,LOW);
  //SPI
  SPI.setRX(SPI_RX); //core already manages which spi to use (using SPI0)
  SPI.setTX(SPI_TX);
  SPI.setSCK(SPI_SCLK);
  SPI.begin();
  SPI.setClockDivider(16);
    //sensors
      //IMU
  imu.setGyroDataRate(IMU_DATA_RATE);
  imu.setAccelDataRate(IMU_DATA_RATE);
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  
  if (!imu.begin_SPI(IMU_CS)){
    Serial.println("IMU did not initialize");
    while(true);
  }
        //configuring interrupts
        //could use wakeup interrupt but not going to bcause lazy
        //config imu speed

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
        
  bool isFound = false; //finding an untaken filename
  for(int i=0; ((!dataFile)&&i<1000); i++){ //CHECK IF THIS IS PROPER, EXAMPLE: https://github.com/earlephilhower/arduino-pico/blob/master/libraries/SD/examples/Datalogger/Datalogger.ino
    dataFile=SD.open("landscaper"+(String)i+".csv",FILE_WRITE);
  }
        //begin file with header
  dataFile.println(""); //TODO:REIMPLEMENT

  //boring peripheral 

  for (int i=0; (i < 5); i++){// initialize reference ground measurements to find altitude change
    getBaroDat();// during flight
    delay(5);
  }
  referenceGroundPressure = roller.recieveNewData('b');//in pascals
  referenceGroundTemperature = roller.receiveNewData('t');// in celsius


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

//Check if test is active (SW1 switched HIGH)
  if (sw1.read()){ //if sw1 = high then rocket is testing
    state=0;
  } else { //else: rocket is armed
    state = 1;
  }
}

void setup1(){ //core 2 setup function
}




void loop() { //Loop 0 - does control loop stuff
  
  prevMillis = millis(); //TODO: add different time variables for different stuff (need to integrate sensor data with different time differences)
  switch (state)
  {
    case 0: //test
    /*
    if (sw2.read()){ //if sw2 = high then cycle thru servo positions
      srvSweep(); //WORRY ABT PARACHUTE SERVO
    }
    if (!sw1.read()){ //arm if mode is swapped
      state = 1;
    }
    */
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
      
    //implement pid later right now analog control (FUCK YEA)
    //implement switch statements later
    //TODO: include roll rate in decision to adjust for roll - to compensate before it's needed
      if (rocketAngle[0] > 0){//roll too much to left
        srvPos[0]+=.1;
        srvPos[1]-=.1;
      }//swap values if neccessary for all of these
      else if (rocketAngle[0] < 0){//roll too much to right
        srvPos[0]-=.1;
        srvPos[1]+=.1;
      }
      if (rocketAngle[2] > 0){//yaw too much to left
        srvPos[0]+=.1;
        srvPos[1]+=.1;
      }
      else if (rocketAngle[2] < 0){//yaw too much to right
        srvPos[0]-=.1;
        srvPos[1]-=.1;
      }
  
      if (rocketAngle[2] > desiredAngle+ROCKET_ANGLE_TOLERANCE){//pitch too much to left
        srvPos[2]+=.1;
        srvPos[3]+=.1;
      }
      else if (rocketAngle[2] < desiredAngle-ROCKET_ANGLE_TOLERANCE){//pitch too much to right
        srvPos[2]-=.1;
        srvPos[3]-=.1;
      }
      

      if (consecMeasurements == 3){//exit loop for when the rocket is at appogee
        state = 3;
        consecMeasurements=0;
      }
      else if (velocityZbaro < 0){
        consecMeasurements++;
      }
      else{
        consecMeasurements = 0;
      }
    //rocketAngle roll pitch yaw
    //predict apogee

    //change servo values

    //check roll data

    //check servo values + adjust if necessary (if over servo threshold, if roll is too high then change one flap angle)

    //check for after apogee
    writeSDData();
    break;

    case 4: //after apogee - when altitude decreases for 15 consecutive measurements - just need to worry about chute deployment
      //make some function predictLandTime(); 
      //if that is over TARGET_TIME  for more than 4 consecutive measurements release the chute (adjust one of the servos)
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
      if (millis() > lastBeepTime + 2000) {
        buzztone(50); // TODO we dont know if buzztone takes time in seconds or in milliseconds
        lastBeepTime = millis();
      }
    break;

  }//end of states
  //send servos to positions
  for (int i=0; i<5; i++){
    if (abs(srvPos[i])>SRV_MAX_ANGLE){
      srvPos[i]=(srvPos[i]/abs(srvPos[i]))*SRV_MAX_ANGLE; 
    }
    srv[i].write((srvPos[i]-srvOffsets[i]));
  }

  getIMUDat();
  getBaroDat();
  
  loopTime=currT-prevMillis;
   //maybe shift to loop1 so loop isn't bogged down

}

long IMUDeltaT; //millis
long altitudeDeltaT; //millis
float IMULastT; 
float altitudeLastT;
float altitudeLast;
float altitude;
void loop1(){ //Core 2 loop - does data filtering when data is available
  // does heavy calculations because calculations are heavy
  //FORMAT NEEDS CHANGE
  
  if(state==2 || state==3 || state==4){

    if(newIMUDat){
    
    //highpass accl

    //highpass gyro

    //lowpass accl 

  //Integrate accl -> velocity, gyro -> pitch angle
      IMUDeltaT = (millis()-IMULastT)/1000;
      
      velocityX+=roller.recieveNewData('X')*IMUDeltaT;//INTEGRATION BABY
      velocityY+=roller.recieveNewData('Y')*IMUDeltaT;
      velocityZ+=roller.recieveNewData('Z')*IMUDeltaT;

      rocketAngle[0]+=roller.recieveNewData('x')*IMUDeltaT;
      rocketAngle[1]+=roller.recieveNewData('y')*IMUDeltaT;
      rocketAngle[2]+=roller.recieveNewData('z')*IMUDeltaT;

      IMULastT = millis();
      newIMUDat = false;
    }
    if(newBaroDat){ //if rocket is inflight do kalman filtering if new data is avaliable 
      //do kalman filtering to get pitch angles
          altitudeDeltaT = (millis() - altitudeLastT)/1000;
          altitude = pressureToAlt(roller.recieveNewData('b'));
          velocityZbaro[0]=velocityZbaro[1];//math wizardry VVV
          velocityZbaro[1]=(altitude-altitudeLast)/altitudeDeltaT; //make work
          float vMagAccl =sqrt((float)(velocityX*velocityX)+(velocityY*velocityX)+(velocityZ*velocityZ));//rocket total velocity
          float pitchEstimateAcclBaro = asin(velocityZbaro[1]/vMagAccl);
          
          altitudeLast = altitude;
          altitudeLastT = millis();
          newBaroDat = false;
          desiredAngle = (100/(1+pow(2.7,-(altitude-200)/25))-1.8);//update desired angle using this equation
    }

}
};


//Sensor interrupt functions, TODO: change them per state (no need for data filtering if on the way down)


void getIMUDat(){
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t tempIMU;
  imu.getEvent(&accel,&gyro,&tempIMU);
  newIMUDat=true;
    
  roller.inputNewData(accel.acceleration.x, 'X');
  roller.inputNewData(accel.acceleration.y, 'Y');
  roller.inputNewData(accel.acceleration.z, 'Z'); 

  roller.inputNewData(gyro.gyro.x, 'x');
  roller.inputNewData(gyro.gyro.y, 'y');
  roller.inputNewData(gyro.gyro.z, 'z'); 
  
  } 

void getBaroDat(){ //when barometric pressure data is available
  newBaroDat=true;
  sensors_event_t pressure;
  sensors_event_t tempBaro;
    
  baro.getEvent(&pressure,&tempBaro);//hecta pascals

  roller.inputNewData(pressure.pressure, 'b');
  roller.inputNewData(tempBaro.temperature, 't');
} 

//Perephrial functions
void srvSweep(){ //sweeps all servoes between 0 degrees and SRV_MAX_POS every SRV_SWEEP_TIME without any delay functions
  currT=millis();
  float srvPosAfterSweep = ((initialSweepMillis-currT)%SRV_SWEEP_TIME)*90; //modulo makes it wrap back around
  for (int i=0; i<4; i++){
    srvPos[i]=srvPosAfterSweep;
  }
}

void buzztone (int time,int frequency = 1000) { //default frequency = 1000 Hz
  tone(BUZZ_PIN,frequency,time);
}

String dataString;
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
  
  dataFile.println(dataString);
  }

//Helper functions 

float pressureToAlt(float pres){ //returns alt (m) from pressure in hecta pascalspascals and temperature in celcies
  return (float)(((273+referenceGroundTemperature)/(-.0065))*((pow((pres/referenceGroundPressure),((8.314*.0065)/(9.807*.02896))))-1)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}

float pid(float *integral, float *previousError, float currentAngle, float desiredAngle, float deltaT){
  float error = desiredAngle-currentAngle;

  float P = Kp * error * deltaT;//porportional

  *integral += error;
  float I = Ki * *integral * deltaT;//integral

  float derrivative = (error-*previousError);
  float D = Kd * derrivative * deltaT;//derrivative
  float output = P+I+D;

  if (output > SRV_MAX_ANGLE){//failsafe
    output = SRV_MAX_ANGLE;
  }
  else if (output < -SRV_MAX_ANGLE){
    output = -SRV_MAX_ANGLE;
  }
  *previousError = error;

  return output;//servo angle
}

unsigned long predictLandTime() {
  //TODO
  //baroAltitude[0] - REF_GROUND_ALTITUDE + velocityZ*t + 0.5*acclZ*t^2
  //t = (-velocityZ*t + sqrt(velocityZ^2-4(height)(0.5*acclZ))/2*height
  float height = baroAltitude[0];
  float a = 0.5 * acclZ[0];
  float b = velocityZ * velocityZ;
  float c = height;
  return (-b + sqrt(b*b-(4*a*c)))/2; //quadratic formula
}
