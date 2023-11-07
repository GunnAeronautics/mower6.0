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
Serial1.println(" ");
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
#define BUZZ_PIN 2 //using tone function


    //Runtime variables
    int spiBeingUsed=false; //to coordinate use of spi
  int state=0;
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
volatile float baroTemp,IMUTemp;
volatile float acclRaw[3],gyroRaw[3];
volatile float baroRaw,tempRaw;
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

class roll{//tested (it works)
  public:
    float accltotal[ROLLING_AVG_LEN];

    void shiftArray(float newData, float* array, int size){
      for (int i=0; i<size-1; i++){ //downshift all values
         array[i]=array[i+1];
      }
      array[size-1]=newData; //replace final value with the new data
      return;
    }

    float getRollingAvg(float array[], int size){
      float sum = 0;
      return ((std::accumulate(array,array+size,sum))/size);
    }

    void inputNewData(float newdata, char datatype){
      switch (datatype) { 
        case 'a': shiftArray(newdata,accltotal,ROLLING_AVG_LEN) ; break;
      
      }
      
    }
    float recieveData(char datatype){
      switch (datatype) {
        case 'a': return getRollingAvg(accltotal,ROLLING_AVG_LEN) ; break;
      }
      return 0;
    }
};
roll roller;//rolling object
void buzztone (int time, int frequency);

void imuIntRoutine();
void baroIntRoutine();

String fname="datalog.csv";
bool isSetUp=false;
void setup() {
  
  // put your setup code here, to run once:
  Serial1.begin(115200);
  Serial1.setRX(1);
  Serial1.setTX(0);
delay(6000);

  // Ensure the SPI pinout the SD card is connected to is configured properly
  SPI.setRX(SPI_RX);
  SPI.setTX(SPI_TX);
  SPI.setSCK(SPI_SCLK);

  imu.begin_SPI(IMU_CS);
  
  imu.setAccelDataRate(IMU_DATA_RATE);
  imu.setGyroDataRate(IMU_DATA_RATE);
  Serial1.println("IMU initialized");
  baro.begin_SPI(BARO_CS);
  baro.setDataRate(BAROMETER_DATA_RATE);
  Serial1.println("Baro initialized");
  Serial1.print("Initializing SD card...");
    if (!SD.begin(SD_CS)) {
    Serial1.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial1.println("SD initialized");
  delay(100);
  fname="datalog"+(String)0+".csv";

  for (int i=0; i<999&&(SD.exists(fname));i++){ //add detection if file already exists
    fname="datalog"+(String)i+".csv";
    Serial1.print("tried ");
    Serial1.println(fname);
  }
  File dataFile = SD.open(fname, FILE_WRITE);
  
  if (!dataFile){
    Serial1.println("dat file not initialized, name = ");
    Serial1.print(fname);
    while (true){
      delay(50);
    }
  } else {
    Serial1.println("dat file successfully initialized, name = ");
    Serial1.print(fname);
  }
  dataFile.println("time, x accl, y accl, z accl, gyro x, gyro y, gyro z, pressure");
  dataFile.close();
  pinMode(BARO_INT,INPUT); //to read dat ready
  pinMode(IMU_INT1,INPUT);
  imu.configIntOutputs(0,0);//ACTIVE LOW
  imu.configInt1(0,1,0);
  baro.configureInterrupt(1,0,1); //ACTIVE LOW
  buzztone(1000,1000);
  delay(1000);
  isSetUp=true;
}

void setup1(){
while(!isSetUp){
delay(1);
}
}

float totalAccel;
void loop() {
  //delay(10);

  switch (state)
  {
  case 0:
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp);
    sensors_event_t temper;
    sensors_event_t pressure;
    baro.getEvent(&pressure, &temper);
    
  
    totalAccel = sqrt(pow(accel.acceleration.x,2)+pow(accel.acceleration.y,2)+pow(accel.acceleration.z,2));
    roller.inputNewData(totalAccel, 'a');
    if (/*roller.recieveNewData('a')*/ totalAccel> ACCEL_THRESH){ 
      state = 1;
      Serial1.println("launch detected, beginning logging");
      /*
      attachInterrupt(BARO_INT,baroIntRoutine,LOW);
      attachInterrupt(IMU_INT1,imuIntRoutine,LOW);
      imu.configIntOutputs(1,0); //active low, pushpull
      imu.configInt1(0,1,1);
      baro.configureInterrupt(0,0,1); //active low, pushpull
      */
      
      
    }
    break;
  // put your main code here, to run repeatedly:
  case 1:
  /*
    totalAccel = sqrt(pow(accel.acceleration.x,2)+pow(accel.acceleration.y,2)+pow(accel.acceleration.z,2));
  roller.inputNewData(totalAccel, 'a');*/
  //noInterrupts(); //protect reading millis and counts
    String dataString = (String)millis() + ',' +
                        (String)acclRaw[0] + ',' +
                        (String)acclRaw[1]  + ',' +
                        (String)acclRaw[2] + ',' +
                        (String)gyroRaw[0] + ',' +
                        (String)gyroRaw[1] + ',' +
                        (String)gyroRaw[2] + ',' +
                        (String)baroRaw;
  Serial1.print("IMU readings since last: ");
  Serial1.println(imuMeasureCount);
  Serial1.print("baro readings since last: ");
  Serial1.println(baroMeasureCount);
  imuMeasureCount=0;
  baroMeasureCount=0;              
  //interrupts();
  Serial1.print("data string: ");
  Serial1.println(dataString);
  
 //Serial1.println("waiting1...");
 while (spiBeingUsed){ //wait your turn :upsidedown:
  delayMicroseconds(10);
  
 }
 spiBeingUsed=true;
  File dataFile = SD.open(fname, FILE_WRITE);
    // if the file is available, write to it:
  if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    
  }
  
  // if the file isn't open, pop up an error:
  else {
    Serial1.print(" error opening ");
    Serial1.println(fname);
  }
  spiBeingUsed=false;
  }
  // print to the serial port too:
    
}
void loop1(){ //reads data if state is 2
uint8_t sensState= ((state<<2)|(digitalReadFast(BARO_INT)<<1)|(digitalReadFast(IMU_INT1)));
  if(sensState<=4&&sensState!=7){ //only read if new imu data is ready, assumes every time theres imu data theres also baro data (lower dat rate) (no function for baro)
  //Serial1.println("waiting2...");
  while (spiBeingUsed){ //wait your turn :upsidedown:
  delayMicroseconds(10);
 }
  spiBeingUsed=true;
  if(!(sensState&1)){ //if imu interrupt goes low
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
  }
  //tempRaw=temp.temperature;
  if (!((sensState>>1)&1)){ //if baro interrupt goes low
  sensors_event_t temper;
  sensors_event_t pressure;
  baro.getEvent(&pressure, &temper);// get pressure
  baroRaw=pressure.pressure;
  }
  spiBeingUsed=false;
  if(!(sensState&1)){
imuMeasureCount++;
  }
  if(!((sensState>>1)&1)){
baroMeasureCount++;
  }
  }

}

void buzztone (int time,int frequency = 1000) { //default frequency = 1000 Hz
  tone(BUZZ_PIN,frequency,time);
}

void imuIntRoutine(){

}
