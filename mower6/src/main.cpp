#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Orientation.h>
#include <Bounce2.h>
#include <Servo.h>
//using adafruit's libraries
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>

//NOTE: the LPS22 is reffered to as baro here, not altimeter because shorter, LSM6DSMXX is reffered to as IMU
//run defines

//SPI frequency currently set by setclockdivider(16) to around 8 MHz
  //NOTE: should be 10MHz (max for imu and altim = 10M)
//#define SPI_FREQ 10000000 

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

  float srvPos[]= new float[4]; //servo position array

  bool newBaroDat = false;
  bool newGyroDat = false;
  bool newAcclDat = false;



  unsigned long initialSweepMillis = 0;


  //raw data
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t pressure;
  sensors_event_t tempIMU;
  sensors_event_t tempBaro;

float acclRaw[][] = new float[3][ROLL_AVG_LEN]; //x, y, z
float gyroRaw[][]= new float[3][ROLL_AVG_LEN];
float baroPressureRaw[] = new float[ROLL_AVG_LEN];

float baroTemp,IMUTemp;

  //filtered data
float acclX,acclY,acclZ;
float gyroR,gyroP,gyroY;
float baroPressure;
float temperature;

  //position variables
Orientation rocketOrientation;
float Roll,Pitch,Yaw;
float altitude;

  //Objects
File dataFile;
Bounce sw1,sw2;
Adafruit_LSM6DS imu;
Adafruit_LPS22 baro;
Servo srv[]=new Servo[4];
//using tone function for buzz

// put function declarations here:
void acclDatRdy(); //interrupt function
void gyroDatRdy(); //interrupt function
void baroDatRdy(); //interrupt function

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
        imu.configInt1(false,false,true); //setting int1 as a gyro ready interrupt
        imu.configInt2(false,true,false); //int2 = accelerometer ready interrupt
        imu.configIntOutputs(true,false); //interrupts = active high, push-pull mode b/c i dont know why you would use open drain
        attachInterrupt(IMU_INT1,gyroDatRdy,HIGH); // check if this is checking if its high or if its inactive high
        attachInterrupt(IMU_INT2,acclDatRdy,HIGH);
        //config speed
        imu.setGyroDataRate(LSM6DS_RATE_208_HZ);
        imu.setAccelDataRate(LSM6DS_RATE_208_HZ);
      //baro
        if (!baro.begin_SPI(BARO_CS)){
          Serial.println("Baro did not initialize");
          while(true);
        }
        //config baro interrupt
        baro.configureInterrupt(false,false,true);
        //config speed
        lps.setDataRate(LPS22_RATE_25_HZ);

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
        dataFile.println("Time (ms),State,Roll,Pitch,Yaw,Alt,accXraw,accYraw,accZraw,gyroRraw,gyroPraw,gyroYraw,baroPressureRaw,temp,loopTime(ms)");

//Buttons
  sw1.attach(CTRL_SW1,INPUT); //attatching debouncer to switches
  sw2.attach(CTRL_SW2,INPUT); 
//Servoes
  srv[0].attach(SRV1_PIN);
  srv[1].attach(SRV2_PIN);
  srv[2].attach(SRV3_PIN);
  srv[3].attach(SRV4_PIN);


buzztone(1,1000); //buzz for perephrial initialization done

//Check if test is active
if (sw1.read()){ //if sw1 = high then rocket is testing
state=0;
} else { //else: rocket is armed
  state = 1;
}

}

void setup1(){

}


void loop() {
  //Interrupt driven code: only record data on SD if new data is ready. if no new data then do nothing,
  prevMillis = millis();
  switch (state)
  {
  case 0: //test


    if (sw2.read()){ //if sw2 = high then cycle thru servo positions
      srvSweep();
    }
    if (!sw1.read()){ //arm if mode is swapped
      state = 1;
    }

    break;
  case 1: //waiting for launch/armed

    break;

  case 2: //launch detected
      //RUN  ONCE: config data rates (wake sensors up)
  
    /* code */
    break;
  case 3: //burnout

    break;

  case 4: //landed
    /* code */
    
    break;
  
  default:
    break;
  }
  currT=millis();
  if (currT<prevMillis){ //accounting for overflow in which case millis will wrap back around to 0
    currT+=(LONG_MAX-prevMillis);
    prevMillis=0;
  }
  for (int i=0; i<4; i++){
    srv[i].write(srvPos[i]);
  }
  loopTime=currT-prevMillis;
  dataFile.println(loopTime+prevMillis+","+(String) state +","+(String)rocketOrientation.toEuler().roll+","+(String)rocketOrientation.toEuler().pitch+","+(String)rocketOrientation.toEuler().yaw+(String)altitude+","+(String)acclXraw+","+(String)acclYraw+","+(String)acclZraw+","+(String)gyroRraw+","+(String)gyroPraw+","+(String)gyroYraw+","+(String)baroPressureRaw+","+(String)acclXraw+","+(String)temperature+","+(String)loopTime);
}

void loop1(){ //loop1 just does data filtering  to turn raw vars into filtered data, its a poor working mf

if(state<4){ //if rocket isn't landed

  if (newAcclDat){ //lowpass
  
  }
  if(newGyroDat){ //hihgpass

  }

  if (newGyroDat&&newAcclDat){ //do kalman shit

  }
  


  if(newBaroDat){ //roll avg

  }
}
if(state ==0){ //listen for servo sweep cmd

}
}




//Sensor interrupt functions
void acclDatRdy(){
  imu.fillAccelEvent(&accel);
  newAcclDat=true;
  //shift vars down in accl raws
  for (int i=0; i<ROLL_AVG_LEN-1; i++){
    for (int j=0; j<3; j++){
      acclRaw[j][i]=acclRaw[j][i+1];
    }
  }
  acclRaw[0][ROLL_AVG_LEN-1] = accel.acceleration.x;
  acclRaw[1][ROLL_AVG_LEN-1] = accel.acceleration.y;
  acclRaw[2][ROLL_AVG_LEN-1] = accel.acceleration.z;

} 
void gyroDatRdy(){
  imu.fillGyroEvent(&gyro);
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
} 
void baroDatRdy(){
  newBaroDat=true;
  baro.getEvent(&baroPressure);
  baroPressureRaw=baroPressure.pressure;
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

