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
/*
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
      return 0;
    }
};

*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.print("Initializing SD card...");

  // Ensure the SPI pinout the SD card is connected to is configured properly
  SPI.setRX(SPI_RX);
  SPI.setTX(SPI_TX);
  SPI.setSCK(SPI_SCLK);


    if (!SD.begin(SD_CS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
    String dataString = ":)";
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}
