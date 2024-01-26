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
#define ACCEL_THRESH 25
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
#define IMU_CS 0//correct
#define SD_CS 21//chip select//correct
#define BARO_CS 22//chip select//correct
//TX = DO = MOSI, RX = DI=MISO
#define SPI_SCLK 18//correct
#define SPI_TX 19 //AKA mosi//correct
#define SPI_RX 20 //AKA miso//correct

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
volatile float baroTemp,IMUTemp;
volatile float acclRaw[3],gyroRaw[3];
float baroRaw,tempRaw;
int imuMeasureCount,baroMeasureCount;

int altitudePoint=0;//cycle through altitude points


  //Objects
File dataFile;
  //Debouncing for switches
Bounce sw1,sw2;

Adafruit_LSM6DS imu;
Adafruit_LPS22 baro;

Servo srv[5];

float originalTemper;
float originalBaro;
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
volatile bool isSetUp=false; //prob being read at the same time by both cores
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
  
Serial.print("IMU initialized, initialization bool = ");Serial.println(imu.begin_SPI(IMU_CS));
  
  imu.setAccelDataRate(IMU_DATA_RATE);
  imu.setGyroDataRate(IMU_DATA_RATE);
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);

  Serial.print("Barometer initialized, initialization bool = ");Serial.println(baro.begin_SPI(BARO_CS));
  baro.setDataRate(BAROMETER_DATA_RATE);

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
  delay(1000);
  isSetUp=true;
  Serial.println("waiting for launch");
  sensors_event_t temper;
  sensors_event_t pressure;
  baro.getEvent(&pressure, &temper);
  originalTemper = temper.temperature;
  originalBaro = pressure.pressure;
}


float pressureToAlt(float pres){ //returns alt (m) from pressure in hecta pascals and temperature in celsius
  return (float)(((273+originalTemper)/(-.0065))*((pow((pres/originalBaro),((8.314*.0065)/(9.807*.02896))))-1)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}

float totalAccel;
void loop() {
  delay(100);
  /*
   sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp); */
    sensors_event_t temper;
    sensors_event_t pressure;
    baro.getEvent(&pressure, &temper);
    
    /*
    acclRaw[0]=accel.acceleration.x;
    acclRaw[1]=accel.acceleration.y;
    acclRaw[2]=accel.acceleration.z;
    gyroRaw[0]=gyro.gyro.x;
    gyroRaw[1]=gyro.gyro.y;
    gyroRaw[2]=gyro.gyro.z;*/
    baroRaw=pressure.pressure;
    float temperature = temper.temperature;
    float altitude = pressureToAlt(baroRaw);
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
    Serial.println((String)millis()+ ' ');
    /*
          Serial.print((String)acclRaw[0]+ ' ');
          Serial.print((String)acclRaw[1]);
          Serial.println((String)gyroRaw[0]+ ' ');
          Serial.print((String)gyroRaw[1]+' ');
          Serial.print((String)gyroRaw[2]+' ');*/
          Serial.println((String)baroRaw+' ');
          Serial.print((String)temperature +' ');
          Serial.print((String)altitude);
    String dataString = (String)millis() + ',' +
                        //(String)acclRaw[0] + ',' +
                        //(String)acclRaw[1]  + ',' +
                        //(String)acclRaw[2] + ',' +
                        //(String)gyroRaw[0] + ',' +
                        //(String)gyroRaw[1] + ',' +
                        //(String)gyroRaw[2] + ',' +
                        (String)baroRaw + ',' +
                        (String)temperature+ ',' +
                        (String)altitude;


  File dataFile = SD.open(fname, FILE_WRITE);
    // if the file is available, write to it:
  if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    
  }  else {
    //Serial.print(" error opening ");
    Serial.println(fname);
  }
    break;
  }

    
}