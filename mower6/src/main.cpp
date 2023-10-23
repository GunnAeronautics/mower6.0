#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Bounce2.h>
#include <Servo.h>
//using adafruit's libraries
#include <Adafruit_LSM6DSOX.h>
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
#define REF_GROUND_PRESSURE 101185.29//in pascals CHANGE BEFORE FLIGHT
#define REF_GROUND_ALTITUDE 30 //in meters CHANGE BEFORE FLIGHT
#define REF_GROUND_TEMPERATURE 17 //in Celsius (must convert to kelvin in code) CHANGE BEFORE FLIGHT
//debug mode adds serial messages and some extra stuff
#define ISDEBUG true


/* template for debug message:

#ifdef ISDEBUG
Serial.println(" ");
#endif

*/
#define ACCEL_THRESH 4
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

  float srvPos[5]; //servo position array
  float srvOffsets[5] = {0,0,0,0,0};
  bool newBaroDat = false;
  bool newGyroDat = false;
  bool newAcclDat = false;

  int8_t consecMeasurements = 0; //this variable should never be greater than 4. Defined as 8-bit integer to save memory



  unsigned long initialSweepMillis = 0;


  //stuff for adafruit's libraries
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t pressure;
  sensors_event_t tempIMU;
  sensors_event_t tempBaro;



long acclDeltaT; //millis
long gyroDeltaT; //millis
long baroDeltaT; //millis
float angleFromIntegration[3];
float baroTemp,IMUTemp;

  //filtered data - keeping past 2 values to enable differentiation
float pitchAngleFiltered;
float acclX[2],acclY[2],acclZ[2];
float velocityX,velocityY,velocityZ;
float velocityZbaro[2];
float gyroRPY[3][2];
float baroAltitude[2]; //keep the past 2 values
float temperature;
float rocketAngle[3];

float altitudeByAngle[3][2] = {
{100,10},
{150,45},
{254,90},
};// change to how many ever points you need to go thru can change later
//data format[x,y] x = altitude, y = angle
//3 is placeholder to whatever
int altitudePoint=0;//cycle through altitude points


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
  //SPI
    SPI.setRX(20); //core already manages which spi to use (using SPI0)
    SPI.setTX(19);
    SPI.setSCK(18);
    SPI.begin();
    SPI.setClockDivider(16);
    //sensors
      //IMU
        if (!imu.begin_SPI(IMU_CS)){
          Serial.println("IMU did not initialize");
          while(true);
        }
        //configuring interrupts
        //could use wakeup interrupt but not going to bcause lazy
        imu.configInt1(false,true,false); //setting int1 as a gyro ready interrupt, as long as both accl and gyro are set @ same rate this shouldnt matter
        imu.configIntOutputs(true,false); //interrupts = active high, push-pull mode b/c i dont know why you would use open drain
        attachInterrupt(IMU_INT1,imuDatRdy,HIGH); // check if this is checking if its high or if its inactive high
        //config imu speed
        imu.setGyroDataRate(IMU_DATA_RATE);
        imu.setAccelDataRate(IMU_DATA_RATE);
      //baro
        if (!baro.begin_SPI(BARO_CS)){
          Serial.println("Baro did not initialize");
          while(true);
        }
        //config baro interrupt
        baro.configureInterrupt(false,false,true);
        //config baro speed
        baro.setDataRate(BAROMETER_DATA_RATE);

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
  //FORMAT NEEDS CHANGE
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
    bool moveToNextState=true;
    for (int i=0; i<ROLLING_AVG_LEN;i++){//check if acceleration is above threshold for all readings
        if((roller.accelRaw[1][i]<ACCEL_THRESH)||(roller.altitudeRaw[i]<ALT_THRESH)){
          moveToNextState=false;
        }
    }
    if (moveToNextState){
      state=2;
    }
    break;

    case 2: //on way up - doing everything (turning n stuff)
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
  
      if (rocketAngle[2] > altitudeByAngle[altitudePoint][1]+ROCKET_ANGLE_TOLERANCE){//pitch too much to left
      srvPos[3]+=.1;
      srvPos[4]+=.1;
      }
      else if (rocketAngle[2] < altitudeByAngle[altitudePoint][1]-ROCKET_ANGLE_TOLERANCE){//pitch too much to right
      srvPos[3]-=.1;
      srvPos[4]-=.1;
      }
      
      if (baroAltitude[1] > altitudeByAngle[altitudePoint][0]){//bro this control system is crazy - implement correction and stuff
        altitudePoint++;//amazing code, change later for optimization
      }
    //rocketAngle roll pitch yaw
    //predict apogee

    //change servo values

    //check roll data

    //check servo values + adjust if necessary (if over servo threshold, if roll is too high then change one flap angle)

    //check for after apogee

    break;

    case 3: //after apogee - when altitude decreases for 15 consecutive measurements - just need to worry about chute deployment
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


    break;

    case 4: //landed - just beep periodically
      if (millis() > lastBeepTime + 2000) {
        buzztone(50); // TODO we dont know if buzztone takes time in seconds or in milliseconds
        lastBeepTime = millis();
      }
    break;
  
  default:

    
    break;
  }
  currT=millis();
  if (currT<prevMillis){ //accounting for overflow in which case millis will wrap back around to 0
    currT+=(LONG_MAX-prevMillis);
    prevMillis=0;
  }
  
  //send servos to positions
  for (int i=0; i<5; i++){
    if (abs(srvPos[i])>SRV_MAX_ANGLE){
      srvPos[i]=(srvPos[i]/abs(srvPos[i]))*SRV_MAX_ANGLE; 
    }
    srv[i].write((srvPos[i]-srvOffsets[i]));
  }
  loopTime=currT-prevMillis;
  //writeSDData(); //maybe shift to loop1 so loop isn't bogged down
}


void loop1(){ //Core 2 loop - does data filtering when data is available
  if(state==2 &&newBaroDat){ //if rocket is inflight do kalman filtering if new data is avaliable 
//do kalman filtering to get pitch angles
  velocityZbaro[1]=velocityZbaro[2];
  velocityZbaro[2]=(baroAltitude[2]-baroAltitude[1])/baroDeltaT; //make work
  float vMagAccl =sqrt((float)(velocityX*velocityX)+(velocityY*velocityX)+(velocityZ*velocityZ));//rocket total velocity
  float pitchEstimateAcclBaro = asin(velocityZbaro[2]/vMagAccl);

  }
  if (state==2 && (newAcclDat&&newGyroDat)){

  //highpass accl

  //highpass gyro

  //lowpass accl 


//Integrate accl -> velocity, gyro -> pitch angle
  velocityX+=acclX[2]*acclDeltaT;
  velocityY+=acclY[2]*acclDeltaT;
  velocityZ+=acclZ[2]*acclDeltaT;

  for (int i=0; i<3;i++){
    angleFromIntegration[i]+=gyroRPY[i][2]*gyroDeltaT; //TODO: correct for true orientation inside of the rocket
  }

}
}



//Sensor interrupt functions, TODO: change them per state (no need for data filtering if on the way down)

void imuDatRdy(){
  imu.getEvent(&accel,&gyro,&tempIMU);
  newGyroDat=true;
    //shift vars down in gyro raws
    
  roller.inputNewData(accel.acceleration.x, 'X');
  roller.inputNewData(accel.acceleration.y, 'Y');
  roller.inputNewData(accel.acceleration.z, 'Z'); 

  roller.inputNewData(gyro.gyro.x, 'x');
  roller.inputNewData(gyro.gyro.y, 'y');
  roller.inputNewData(gyro.gyro.z, 'z'); 

} 



void baroDatRdy(){ //when barometric pressure data is available
 //maybe do different roll avg thing for baro, much less data
  newBaroDat=true;
  baro.getEvent(&pressure,&tempBaro);

  roller.inputNewData(pressureToAlt(pressure.pressure), 'b'); 
  //Convert to altitude
  
  //get avg pressure from rolling avg 

} 

//Perephrial functions
void srvSweep(){ //sweeps all servoes between 0 degrees and SRV_MAX_POS every SRV_SWEEP_TIME without any delay functions
  currT=millis();
  float srvPosAfterSweep = ((initialSweepMillis-currT)%SRV_SWEEP_TIME)*90; //modulo makes it wrap back around
  for (int i=0; i<4; i++){
    srvPos[i]=srvPosAfterSweep;
  }
}
void buzztone (int time,int frequency = 1000){ //default frequency = 1000 Hz
  tone(BUZZ_PIN,frequency,time);
}

void writeSDData (){
  dataFile.println();//TODO: REIMPLEMENT
}

//Helper functions

float pressureToAlt(float pres){ //returns alt (m) from pressure in pascals
  return (float)(REF_GROUND_ALTITUDE+((273+REF_GROUND_TEMPERATURE)/(-.0065))*((pow((pres/REF_GROUND_PRESSURE),((8.314*.0065)/(9.807*.02896))))-1)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}

unsigned long predictLandTime() {
  //TODO
  //baroAltitude[0] - REF_GROUND_ALTITUDE + velocityZ*t + 0.5*acclZ*t^2
  //t = (-velocityZ*t + sqrt(velocityZ^2-4(height)(0.5*acclZ))/2*height
  float height = baroAltitude[0] - REF_GROUND_ALTITUDE;
  float a = 0.5 * acclZ[0];
  float b = velocityZ * velocityZ;
  float c = height;
  return (-b + sqrt(b*b-(4*a*c)))/2; //quadratic formula
}