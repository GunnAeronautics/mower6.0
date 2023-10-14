#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Bounce2.h>
#include <Servo.h>
//using adafruit's libraries
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
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
#define SRV_MAX_ANGLE 7 //in degrees

#define IMU_DATA_RATE LSM6DS_RATE_208_HZ
#define BAROMETER_DATA_RATE LPS22_RATE_25_HZ

//Length of rolling avg arrays
#define ROLL_AVG_LEN 5

#define SRV_SWEEP_TIME 2500//in millis
#define SRV_MAX_POS 90 //degrees

//Pin defs
#define SRV1_PIN 4
#define SRV2_PIN 5
#define SRV3_PIN 6
#define SRV4_PIN 7
#define IMU_INT1 10
#define IMU_INT2 11
#define BARO_INT 12
#define DEBUG_LED 13
#define IMU_CS 15
#define SD_CS 16
//TX = DO = MOSI, RX = DI=MISO
#define SPI_SCLK 18
#define SPI_TX 19
#define SPI_RX 20
#define BARO_CS 21
#define CTRL_SW1 22
#define CTRL_SW2 23
#define BUZZ_PIN 24 //using tone function


    //Runtime variables
  int state=0;
  unsigned long prevMillis=0;
  unsigned long currT = 0;
  int loopTime = 0;

  float srvPos[4]; //servo position array

  bool newBaroDat = false;
  bool newGyroDat = false;
  bool newAcclDat = false;



  unsigned long initialSweepMillis = 0;


  //stuff for adafruit's libraries
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t pressure;
  sensors_event_t tempIMU;
  sensors_event_t tempBaro;


float acclRaw[3][ROLL_AVG_LEN]; //x, y, z
long acclDeltaT; //millis
float gyroRaw[3][ROLL_AVG_LEN]; //r, p, y
long gyroDeltaT; //millis
float baroPressureRaw[ROLL_AVG_LEN];
long baroDeltaT; //millis
float pitchAngleFromIntegration;
float baroTemp,IMUTemp;

  //filtered data - keeping past 2 values to enable differentiation
float pitchAngleFiltered;
float acclX[2],acclY[2],acclZ[2];
float velocityX,velocityY,velocityZ;
float velocityZbaro[2];
float gyroR[2],gyroP[2],gyroY[2];
float baroAltitude[2]; //keep the past 2 values
float temperature;


  //Objects
File dataFile;
  //Debouncing for switches
Bounce sw1,sw2;

Adafruit_LSM6DS imu;
Adafruit_LPS22 baro;

Servo srv[4];

//using tone function for buzz

  //interrupt function for when imu data is ready
void imuDatRdy(); 
  //interrupt function for when barometer data is ready
void baroDatRdy(); 
  //Buzzes a tone at frequency in Hz and for time in ms (i think)
void buzztone (int frequency,int time);



void setup() {

//communication interface begins
    Serial.begin();
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
        dataFile.println("Time (ms),State,Pitch,Alt,accXraw,accYraw,accZraw,gyroRraw,gyroPraw,gyroYraw,baroPressureRaw,temp,loopTime(ms)");

  //boring peripheral 

//attatching switches to debouncers
  sw1.attach(CTRL_SW1,INPUT); //attaching debouncer to switches
  sw2.attach(CTRL_SW2,INPUT); 
//Servos
  srv[0].attach(SRV1_PIN);
  srv[1].attach(SRV2_PIN);
  srv[2].attach(SRV3_PIN);
  srv[3].attach(SRV4_PIN);


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


void loop() { //Loop 1 - does control loop stuff
  //FORMAT NEEDS CHANGE
  prevMillis = millis(); //TODO: add different time variables for different stuff (need to integrate sensor data with different time differences)
  switch (state)
  {
  case 0: //test


    if (sw2.read()){ //if sw2 = high then cycle thru servo positions
      srvSweep(); //WORRY ABT PARACHUTE SERVO
    }
    if (!sw1.read()){ //arm if mode is swapped
      state = 1;
    }


    break;

    case 1://on pad - waiting for high acceleration, major change in pressure TODO: maybe configure one of the interrupts on imu for wakeup signal, (or 6d interrupt)
    break;

    case 2: //on way up - doing everything (turning n stuff)
    //predict apogee

    //change servo values

    //check roll data

    //check servo values + adjust if necessary (if over servo threshold, if roll is too high then change one flap angle)

    //check for after apogee

    break;

    case 3: //after apogee - when altitude decreases for 15 consecutive measurements - just need to worry about chute deployment
    break;

    case 4: //landed - just beep periodically
      if (millis() % 2 == 1) buzztone(50); // this probably doesnt work idk how to test it tbh
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
  for (int i=0; i<4; i++){
    srv[i].write(srvPos[i]);
  }
  loopTime=currT-prevMillis;
  //writeSDData();
}


void loop1(){ //Core 2 loop - does data filtering when data is available
  if(state==2 &&newBaroDat){ //if rocket is inflight do kalman filtering if new data is avaliable 
//do kalman filtering to get pitch angles
  velocityZbaro[1]=velocityZbaro[2];
  velocityZbaro[2]=(baroAltitude[2]-baroAltitude[1])/baroDeltaT; //make work
  float vMagAccl =sqrt((float)(velocityX*velocityX)+(velocityY*velocityX)+(velocityZ*velocityZ));
  float pitchEstimateAcclBaro = asin(velocityZbaro[2]/vMagAccl);

  }
  if (state==2 && (newAcclDat&&newGyroDat)){
//filter all data (highpass and lowpass) -> complementary filter (TODO)

//rolling avg
float tempAcc[3],tempGyro[3];
  for (int i=0; i<ROLL_AVG_LEN-1;i++){
    for (int j=0; j<3; j++){
      tempAcc[j]+=acclRaw[j][i];
      tempGyro[j]+=gyroRaw[j][i];
    }
  }
  tempAcc[0]/=ROLL_AVG_LEN;
  tempAcc[1]/=ROLL_AVG_LEN;
  tempAcc[2]/=ROLL_AVG_LEN;
  tempGyro[0]/=ROLL_AVG_LEN;
  tempGyro[1]/=ROLL_AVG_LEN;
  tempGyro[2]/=ROLL_AVG_LEN;

  //highpass accl

  //highpass gyro

  //lowpass accl 


//Integrate accl -> velocity, gyro -> pitch angle
  velocityX+=acclX[2]*acclDeltaT;
  velocityY+=acclY[2]*acclDeltaT;
  velocityZ+=acclZ[2]*acclDeltaT;

  pitchAngleFromIntegration+=gyroP[2]*gyroDeltaT; //TODO: correct for true orientation inside of the rocket

  }

}




//Sensor interrupt functions, TODO: change them per state (no need for data filtering if on the way down)

void imuDatRdy(){
  imu.getEvent(&accel,&gyro,&tempIMU);
  newGyroDat=true;
    //shift vars down in gyro raws
  for (int i=0; i<ROLL_AVG_LEN-1; i++){
    for (int j=0; j<3; j++){
      gyroRaw[j][i]=gyroRaw[j][i+1];
    }
  }
  gyroRaw[0][ROLL_AVG_LEN-1] = gyro.gyro.x;
  gyroRaw[1][ROLL_AVG_LEN-1] = gyro.gyro.y;
  gyroRaw[2][ROLL_AVG_LEN-1] = gyro.gyro.z;
  for (int i=0; i<ROLL_AVG_LEN-1; i++){
    for (int j=0; j<3; j++){
      acclRaw[j][i]=acclRaw[j][i+1];
    }
  }
  acclRaw[0][ROLL_AVG_LEN-1] = accel.acceleration.x;
  acclRaw[1][ROLL_AVG_LEN-1] = accel.acceleration.y;
  acclRaw[2][ROLL_AVG_LEN-1] = accel.acceleration.z;
} 



void baroDatRdy(){ //when barometric pressure data is available
 //maybe do different roll avg thing for baro, much less data
  newBaroDat=true;
  baro.getEvent(&pressure);
  for (int i=0; i<ROLL_AVG_LEN-1; i++){      baroPressureRaw[i]=baroPressureRaw[i+1];
  }
  baroPressureRaw[ROLL_AVG_LEN-1]=pressure.pressure;
  //Convert to altitude

  //get avg pressure from rolling avg 

} 

//Perephrial functions
void srvSweep(){ //sweeps all servoes between 0 degrees and SRV_MAX_POS every SRV_SWEEP_TIME without any delay functions
  currT=millis();
  float srvPosAfterSweep = ((initialSweepMillis-currT)%SRV_SWEEP_TIME)*SRV_MAX_POS; //modulo makes it wrap back around
  for (int i=0; i<4; i++){
    srvPos[i]=srvPosAfterSweep;
  }
}
void buzztone (int time,int frequency = 1000){ //default frequency = 1000 Hz
  tone(BUZZ_PIN,frequency,time);
}

void writeSDData (){
  dataFile.println(loopTime+prevMillis+","+(String) state +","+(String)pitchAngleFiltered+","+(String)baroAltitude[2]+","+(String)acclRaw[0][ROLL_AVG_LEN-1]+","+(String)acclRaw[1][ROLL_AVG_LEN-1]+","+(String)acclRaw[2][ROLL_AVG_LEN-1]+","+(String)gyroRaw[0][ROLL_AVG_LEN-1]+","+(String)gyroRaw[1][ROLL_AVG_LEN-1]+","+(String)gyroRaw[2][ROLL_AVG_LEN-1]+","+(String)baroPressureRaw[ROLL_AVG_LEN-1]+","+(String)temperature+","+(String)loopTime);
}

//Helper functions

float pressureToAlt(float pres){ //returns alt (m) from pressure in pascals
  return (float)(REF_GROUND_ALTITUDE+((273+REF_GROUND_TEMPERATURE)/(-.0065))*((pow((pres/REF_GROUND_PRESSURE),((8.314*.0065)/(9.807*.02896))))-1)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}
