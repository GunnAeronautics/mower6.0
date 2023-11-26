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
#define ACCEL_THRESH 15
#define ALT_THRESH 25 //meters above initial
#define SRV_MAX_ANGLE 7 //in degrees
#define ROCKET_ANGLE_TOLERANCE 5 //in degrees

#define IMU_DATA_RATE LSM6DS_RATE_104_HZ
#define BAROMETER_DATA_RATE LPS22_RATE_75_HZ

//amount of rolling average numbers to keep track of
#define ROLLING_AVG_LEN 7

#define SRV_SWEEP_TIME 2500//in millis

//Pin defs 
#define SRV1_PIN 4
#define SRV2_PIN 5
#define SRV3_PIN 6
#define SRV4_PIN 7
#define SRV5_PIN 8
#define IMU_INT1 10
#define IMU_INT2 12
#define BARO_INT 13
#define DEBUG_LED 25
#define IMU_CS 9
#define SD_CS 14
//TX = DO = MOSI, RX = DI=MISO
#define SPI_SCLK 18
#define SPI_TX 19 //AKA mosi
#define SPI_RX 20 //AKA miso
#define BARO_CS 15
#define CTRL_SW1 26
#define CTRL_SW2 22
#define BUZZ_PIN 3 //using tone function


    //Runtime variables
    int spiBeingUsed=false; //to coordinate use of spi
  int state=1;
  unsigned long prevMillis=0;
  unsigned long currT = 0;
  int loopTime = 0;
  unsigned long lastBeepTime = 0; //the last time the beep happened during case 4 (beep)

  float srvPos[5]; //servo position array
  float srvOffsets[5] = {0,0,0,0,0};

  int8_t consecMeasurements = 0; //this variable should never be greater than 4. Defined as 8-bit integer to save memory
  unsigned long initialSweepMillis = 0;

long acclDeltaT; //millis
long gyroDeltaT; //millis
long baroDeltaT; //millis
float angleFromIntegration[3];
float baroTemp,IMUTemp;
float acclRaw[3],gyroRaw[3];
float baroRaw,tempRaw,baroTempRaw;
float altitude;
int imuMeasureCount,baroMeasureCount;

int altitudePoint=0;//cycle through altitude points


  //Objects
File dataFile;
  //Debouncing for switches
Bounce sw1,sw2;

Adafruit_LSM6DS imu;
Adafruit_LPS22 baro;

Servo srv[5];

//rolling average

void buzztone (int time, int frequency);

void imuIntRoutine();
void baroIntRoutine();
String dataString;
String fname="datalog.csv";
float referenceGroundPressure;
float referenceGroundTemperature;

void setup() {
  
  // put your setup code here, to run once:
  
  //Serial.setRX(1);
  //Serial.setTX(0);
  
  Serial.begin(115200);
delay(6000);
pinMode(LED_BUILTIN,OUTPUT);

  // Ensure the SPI pinout the SD card is connected to is configured properly
  SPI.setRX(SPI_RX);
  SPI.setTX(SPI_TX);
  SPI.setSCK(SPI_SCLK);
      digitalWrite(LED_BUILTIN,HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN,LOW);
           delay(200);
  bool isIMU = imu.begin_SPI(IMU_CS); 
  if(!isIMU){
    while(true){}
  }
  imu.setAccelDataRate(IMU_DATA_RATE);
  imu.setGyroDataRate(IMU_DATA_RATE);
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  
  Serial.println("IMU initialized");
        digitalWrite(LED_BUILTIN,HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN,LOW);
           delay(200);
  bool isBaro = baro.begin_SPI(BARO_CS);
  baro.setDataRate(BAROMETER_DATA_RATE);
        digitalWrite(LED_BUILTIN,HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN,LOW);
           delay(200);
  Serial.println("Baro initialized");
  Serial.print("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //while(true){}
    //return;
  }
  Serial.println("SD initialized");
  delay(100);
  fname="datalog"+(String)0+".csv";
      digitalWrite(LED_BUILTIN,HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN,LOW);
           delay(200);
  for (int i=0; i<999&&(SD.exists(fname));i++){ //add detection if file already exists
    fname="datalog"+(String)i+".csv";
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
  dataFile.println("time, x accl, y accl, z accl, gyro x, gyro y, gyro z, pressure");
  dataFile.close();
  /*
  pinMode(BARO_INT,INPUT); //to read dat ready
  pinMode(IMU_INT1,INPUT);
  imu.configIntOutputs(0,0);//ACTIVE LOW
  imu.configInt1(0,1,0);
  baro.configureInterrupt(1,0,1); //ACTIVE LOW */
  //buzztone(1000,1000);
  delay(1000);
  
  for (int i=0; (i < 5); i++){// initialize reference ground measurements to find altitude change
  sensors_event_t pressure;
  sensors_event_t tempBaro;
    
  baro.getEvent(&pressure,&tempBaro);//hecta pascals

  baroRaw = pressure.pressure;
  baroTempRaw = tempBaro.temperature;
    referenceGroundPressure += baroRaw;
    referenceGroundTemperature += baroTempRaw;
    delay(100);
  }
  referenceGroundPressure /= 5;
  referenceGroundTemperature /= 5;
}
/*
void setup1(){
while(!isSetUp){}//the cores arent sharing

//Serial.println("amogus");
}*/

float totalAccel;
float lastAltitude;
float altitudeVelocity;
float deltaT;
float lastT;
void loop() {
  //delay(10);
  deltaT = (millis()-lastT)/1000
   sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp);

    acclRaw[0]=accel.acceleration.x;
    acclRaw[1]=accel.acceleration.y;
    acclRaw[2]=accel.acceleration.z;
    gyroRaw[0]=gyro.gyro.x;
    gyroRaw[1]=gyro.gyro.y;
    gyroRaw[2]=gyro.gyro.z;

    sensors_event_t pressure;
  sensors_event_t tempBaro;
    
  baro.getEvent(&pressure,&tempBaro);//hecta pascals

  baroRaw = pressure.pressure;
  baroTempRaw = tempBaro.temperature;
  altitude =(((273+referenceGroundTemperature)/(-.0065))*((pow((baroRaw/referenceGroundPressure),((8.314*.0065)/(9.807*.02896))))-1));
  altitudeVelocity = (altitude-lastAltitude) * deltaT;
  switch (state){
  case 0:  
    totalAccel = sqrt(pow(acclRaw[0],2)+pow(acclRaw[1],2)+pow(acclRaw[2],2));
    roller.inputNewData(totalAccel, 'a');
    if (/*roller.recieveNewData('a')*/ totalAccel> ACCEL_THRESH){ 
      state = 1;
      Serial.println("launch detected, beginning logging");
      

     pinMode(LED_BUILTIN,OUTPUT);
      digitalWrite(LED_BUILTIN,HIGH);
      //delay(40);
      
    }
    break;
  // put your main code here, to run repeatedly:
  case 1:
    if (consecMeasurements == 3){//exit loop for when the rocket above 100 meters
        state = 2;
        consecMeasurements=0;
      }
      else if (altitude>100){// if rocket over 100 meters up then do thing
        consecMeasurements++;
      }
      else{
        consecMeasurements = 0;
      }

    writeSDData()
    break;
  case 2:
    if (consecMeasurements == 3){//exit loop for when the rocket above 100 meters
            state = 3;
            consecMeasurements=0;
          }
          else if (altitudeVelocity < 0){// if rocket over 100 meters up then do thing
            consecMeasurements++;
          }
          else{
            consecMeasurements = 0;
          }
    writeSDData()
    break;
  case 3;
    writeSDData()
  lastT = millis()
  lastAltitude = altitude()
  /*
    totalAccel = sqrt(pow(accel.acceleration.x,2)+pow(accel.acceleration.y,2)+pow(accel.acceleration.z,2));
  roller.inputNewData(totalAccel, 'a');*/
  //noInterrupts(); //protect reading millis and counts

  //Serial.print("IMU readings since last: ");
  //Serial.println(imuMeasureCount);
  //Serial.print("baro readings since last: ");
  //Serial.println(baroMeasureCount);
  //imuMeasureCount=0;
  //baroMeasureCount=0;             
  //interrupts();
  //Serial.print("data string: ");
  //Serial.println(dataString);

  /*
 //Serial.println("waiting1...");
 while (spiBeingUsed){ //wait your turn :upsidedown:
  delayMicroseconds(10);
  
 }
 spiBeingUsed=true;*/

  // print to the serial port too:

  }
}


void writeSDData(){
  File dataFile = SD.open(fname, FILE_WRITE);
  dataString =          (String)millis() + ',' +
                        (String)acclRaw[0] + ',' +
                        (String)acclRaw[1]  + ',' +
                        (String)acclRaw[2] + ',' +
                        (String)gyroRaw[0] + ',' +
                        (String)gyroRaw[1] + ',' +
                        (String)gyroRaw[2] + ',' +
                        (String)baroRaw + ',' +
                        (String)altitude + ',' +
                        (String)state;
  if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
      
    }  else {
      //Serial.print(" error opening ");
      Serial.println(fname);
    }

}



void buzztone (int time,int frequency = 1000) { //default frequency = 1000 Hz
  tone(BUZZ_PIN,frequency,time);
}
