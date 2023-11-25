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
<<<<<<< Updated upstream
=======


>>>>>>> Stashed changes
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

  float srvPos[5]; //servo position array
  float srvOffsets[5] = {0,0,0,0,0};
<<<<<<< Updated upstream
  bool newBaroDat = false;
  bool newIMUDat = false;
=======
  volatile bool newSensorDat = false;
>>>>>>> Stashed changes

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
float rocketAngle[3];//integrated

float altitudeByAngle[3][2] = {
{0,  0},
{130,10}
{150,40},
{170,50},
{190,60},
{210,60},
{230,70},
{245,90},
{250,85},
{999,90}//if it goes above then point DOWN
};// change to how many ever points you need to go thru can change later
//data format[x,y] x = altitude, y = angle
//3 is placeholder to whatever
int altitudePoint = 0;//cycle through altitude points


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
        case 'X': shiftArray(newdata,acclRaw[0],ROLLING_AVG_LEN) ; break;
        case 'Y': shiftArray(newdata,acclRaw[1],ROLLING_AVG_LEN) ; break;
        case 'Z': shiftArray(newdata,acclRaw[2],ROLLING_AVG_LEN) ; break;
        case 'x': shiftArray(newdata,gyroRaw[0],ROLLING_AVG_LEN) ; break;
        case 'y': shiftArray(newdata,gyroRaw[1],ROLLING_AVG_LEN) ; break;
        case 'z': shiftArray(newdata,gyroRaw[2],ROLLING_AVG_LEN) ; break;
        case 'b': shiftArray(newdata,baroRaw,ROLLING_AVG_LEN)    ; break;
        case 't': shiftArray(newdata,baroTempRaw,ROLLING_AVG_LEN); break;
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
        case 'b': return getAvgInRollingAvg(baroRaw,ROLLING_AVG_LEN) ; break;
        case 't': return getAvgInRollingAvg(baroTempRaw,ROLLING_AVG_LEN) ; break;
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
  //SPI
<<<<<<< Updated upstream
  SPI.setRX(20); //core already manages which spi to use (using SPI0)
  SPI.setTX(19);
  SPI.setSCK(18);
  SPI.begin();
  SPI.setClockDivider(16);
    //sensors
      //IMU
=======
  SPI.setRX(SPI_RX); //core already manages which spi to use (using SPI0)
  SPI.setTX(SPI_TX);
  SPI.setSCK(SPI_SCLK);
  SPI.begin(); //might matter, needs to be tested in action
    //sensors
      //IMU

  
>>>>>>> Stashed changes
  if (!imu.begin_SPI(IMU_CS)){
    Serial.println("IMU did not initialize");
    while(true);
  }
<<<<<<< Updated upstream
        //configuring interrupts
        //could use wakeup interrupt but not going to bcause lazy
  imu.configInt1(false,true,false); //setting int1 as a gyro ready interrupt, as long as both accl and gyro are set @ same rate this shouldnt matter
  imu.configIntOutputs(true,false); //interrupts = active high, push-pull mode b/c i dont know why you would use open drain
  attachInterrupt(IMU_INT1,imuDatRdy,HIGH); // check if this is checking if its high or if its inactive high
        //config imu speed
  imu.setGyroDataRate(IMU_DATA_RATE);
  imu.setAccelDataRate(IMU_DATA_RATE);
=======
        //config imu speed
  imu.setGyroDataRate(IMU_DATA_RATE);
  imu.setAccelDataRate(IMU_DATA_RATE);
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS); //should be sufficient
>>>>>>> Stashed changes
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
    baroDatRdy();// during flight
    delay(5);
  }
  float referenceGroundPressure = recieveNewData('b');//in pascals
  float referenceGroundTemperature = receiveNewData('t');// in celsius

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
<<<<<<< Updated upstream
  //FORMAT NEEDS CHANGE
  prevMillis = millis(); //TODO: add different time variables for different stuff (need to integrate sensor data with different time differences)
=======
  getIMUDat();
  getBaroDat();
  prevSensorMillis = millis();

>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
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
        srvPos[2]+=.1;
        srvPos[3]+=.1;
      }
      else if (rocketAngle[2] < altitudeByAngle[altitudePoint][1]-ROCKET_ANGLE_TOLERANCE){//pitch too much to right
        srvPos[2]-=.1;
        srvPos[3]-=.1;
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
=======
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
    writeSDData();
    break;

    case 4: //after apogee - when altitude decreases for 15 consecutive measurements - just need to worry about chute deployment
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
    case 4: //landed - just beep periodically
      if (millis() > lastBeepTime + 2000) {
=======
    case 5: //landed - just beep periodically
      if (millis() > lastBeepTime + 2000) { //FIX MAX MILLIS CASE
>>>>>>> Stashed changes
        buzztone(50); // TODO we dont know if buzztone takes time in seconds or in milliseconds
        lastBeepTime = millis();
      }
    break;
  
  default:

<<<<<<< Updated upstream
    
    break;
  }
  currT=millis();
  if (currT<prevMillis){ //accounting for overflow in which case millis will wrap back around to 0
    currT+=(LONG_MAX-prevMillis);
    prevMillis=0;
  }
  
=======
  }//end of states

>>>>>>> Stashed changes
  //send servos to positions
  for (int i=0; i<5; i++){
    #ifdef ISCANARD
    if ((abs(srvPos[i])>SRV_MAX_ANGLE)&&i<5){ //clop srv angle if canarding and not the dual deployment servo
      srvPos[i]=(srvPos[i]/abs(srvPos[i]))*SRV_MAX_ANGLE; 
    }
    #endif
    srv[i].write((srvPos[i]-srvOffsets[i]));
  }
<<<<<<< Updated upstream
  loopTime=currT-prevMillis;
  //writeSDData(); //maybe shift to loop1 so loop isn't bogged down
}


long IMUDeltaT; //millis
long altitudeDeltaT; //millis
float IMULastT; 
float altitudeLastT;
float altitudeLast;
float altitude;
void loop1(){ //Core 2 loop - does data filtering when data is available
  // unused for now because it doesn't do anything re implement later
  if(state==2 &&newBaroDat){ //if rocket is inflight do kalman filtering if new data is avaliable 
//do kalman filtering to get pitch angles
    altitudeDeltaT = (millis() - altitudeLastT)/1000;
    altitude = pressureToAlt(roller.recieveNewData('b'))
    velocityZbaro[0]=velocityZbaro[1];//math wizardry VVV
    velocityZbaro[1]=(altitude-altitudeLast)/altitudeDeltaT; //make work
    float vMagAccl =sqrt((float)(velocityX*velocityX)+(velocityY*velocityX)+(velocityZ*velocityZ));//rocket total velocity
    float pitchEstimateAcclBaro = asin(velocityZbaro[1]/vMagAccl);
=======


}

int sensorDeltaT;
volatile long prevSensorMillis;
float altitude[2];
void loop1(){ //Core 2 loop - does data filtering when data is available
  // does heavy calculations because calculations are heavy
  /*TODO:
  add way to turn raw variables into "actual form"
  take gravity impact into account on integration
  */
  if(state==2 || state==3 || state==4){ //needed

    if(newSensorDat){
>>>>>>> Stashed changes
    
    altitudeLast = altitude;
    altitudeLastT = millis();
    newBaroDat = False;
  }
  if (state==2 && newIMUDat){
  
  //highpass accl

  //highpass gyro

<<<<<<< Updated upstream
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
    newIMUDat = False;
  }
=======


  //Integrate accl -> velocity, gyro -> pitch angle
      sensorDeltaT = (millis()-prevSensorMillis)/1000;
      
      velocityX+=roller.recieveNewData('X')*sensorDeltaT;//bad integration - needs clipping (? - measure impact at some point)
      velocityY+=roller.recieveNewData('Y')*sensorDeltaT;
      velocityZ+=roller.recieveNewData('Z')*sensorDeltaT; 

      rocketAngle[0]+=roller.recieveNewData('x')*sensorDeltaT;
      rocketAngle[1]+=roller.recieveNewData('y')*sensorDeltaT;
      rocketAngle[2]+=roller.recieveNewData('z')*sensorDeltaT;
      
    
    
      //do kalman filtering to get pitch angles
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
          #endif
          prevSensorMillis = millis();
          newSensorDat = false;
          //update desired angle using this equation
    }
}
>>>>>>> Stashed changes
}


//Sensor interrupt functions, TODO: change them per state (no need for data filtering if on the way down)


void imuDatRdy(){
  imu.getEvent(&accel,&gyro,&tempIMU);
  newSensorDat=true;
    
  roller.inputNewData(accel.acceleration.x, 'X');
  roller.inputNewData(accel.acceleration.y, 'Y');
  roller.inputNewData(accel.acceleration.z, 'Z'); 

  roller.inputNewData(gyro.gyro.x, 'x');
  roller.inputNewData(gyro.gyro.y, 'y');
  roller.inputNewData(gyro.gyro.z, 'z');
  
  } 

<<<<<<< Updated upstream
void baroDatRdy(){ //when barometric pressure data is available
  newBaroDat=true;
=======
void getBaroDat(){ //when barometric pressure data is available
newSensorDat=true;
  sensors_event_t pressure;
  sensors_event_t tempBaro;
    
>>>>>>> Stashed changes
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

String dataString;
void writeSDData (){
<<<<<<< Updated upstream
  dataFile.print(roller.recieveRawData('X'));dataFile.print(roller.recieveRawData(','));
  dataFile.print(roller.recieveRawData('Y'));dataFile.print(roller.recieveRawData(','));
  dataFile.print(roller.recieveRawData('X'));dataFile.print(roller.recieveRawData(','));
  dataFile.print(roller.recieveRawData('y'));dataFile.print(roller.recieveRawData(','));
  dataFile.print(roller.recieveRawData('z'));dataFile.print(roller.recieveRawData(','));
  dataFile.print(roller.recieveRawData('b'));dataFile.print(roller.recieveRawData(','));
  dataFile.println(roller.recieveRawData('t'));
}
=======
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
>>>>>>> Stashed changes


//Helper functions 

float pressureToAlt(float pres){ //returns alt (m) from pressure in hecta pascals and temperature in celsius
  return (float)(((273+referenceGroundTemperature)/(-.0065))*((pow((pres/referenceGroundPressure),((8.314*.0065)/(9.807*.02896))))-1)); //https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
}

<<<<<<< Updated upstream
unsigned long predictLandTime() {
  //TODO
  //baroAltitude[0] - REF_GROUND_ALTITUDE + velocityZ*t + 0.5*acclZ*t^2
  //t = (-velocityZ*t + sqrt(velocityZ^2-4(height)(0.5*acclZ))/2*height
  float height = baroAltitude[0] - REF_GROUND_ALTITUDE;
  float a = 0.5 * acclZ[0];
=======

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
>>>>>>> Stashed changes
  float b = velocityZ * velocityZ;
  float c = height;
  return (-b + sqrt(b*b-(4*a*c)))/2; //quadratic formula
}

<<<<<<< Updated upstream
=======
#ifdef CANARD //canard speciifc helpers and variables

#define ROLLKp .1
#define ROLLKi .1
#define ROLLKd .1
#define PITCHKp .1
#define PITCHKi .1
#define PITCHKd .1
#define YAWKp .1
#define ROLLKi .1
#define ROLLKd .1

#define SERVOCOEFFICIENT .01 //output of pid is multiplied by this to get the servo angle

#define PITCHFUNCTION (100/(1+pow(2.7,-((altitude[1]-200)/25)))-1.8)

float rollIntegral,pitchIntegral,yawIntegral;
float rollError,pitchError,yawError;


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

//PID 
float predictApogee (float currV,float k){ // finds the distance to max alt, takes v and k - ONLY VALID FOR COASTING
//
return (MASS/(2*CURRK) )*log((MASS*9.81+velocityZ*velocityZ*CURRK)/(MASS*9.81));
} //https://www.rocketmime.com/rockets/qref.html for range equation
 
float inverseApogee(float desiredApogee){ //returns a k given desired apogee, if unattainable then return -1, uses eulers method
  float currX,currSlope,currPrediction, currStepSize;
  for (float n=0; !(currPrediction<desiredApogee-INVERSE_APOGEE_TOLERANCE&&currPrediction>desiredApogee+INVERSE_APOGEE_TOLERANCE)&&n<40; n++){ //could implement dynamic step size with derivatives + lin approximations
    currPrediction=predictApogee(velocityZ ,currX); //chose if we want to use velocityz or velocityzbaro
    currSlope=(predictApogee(velocityZ,currX+.01)-currPrediction)/.01;
    currStepSize=-(currX)+(desiredApogee-currPrediction)/currSlope; //y = currSlope*currX+currPrediction, step+currx = (desiredApogee-currPrediction)/currSlope
    currX+=currStepSize;
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
>>>>>>> Stashed changes
