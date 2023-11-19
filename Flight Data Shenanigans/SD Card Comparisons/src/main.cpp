#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Bounce2.h>

//using adafruit's libraries
#include <Adafruit_Sensor.h>
#include <numeric>

#define DEBUG_LED 25
#define SD_CS 14
//TX = DO = MOSI, RX = DI=MISO
#define SPI_SCLK 18
#define SPI_TX 19 //AKA mosi
#define SPI_RX 20 //AKA miso
#define CTRL_SW1 26
#define CTRL_SW2 22
#define BUZZ_PIN 3 //using tone function


    //Runtime variables

  int state=0;
  unsigned long prevMillis=0;
  unsigned long currT = 0;
  int loopTime = 0;
  unsigned long lastBeepTime = 0; //the last time the beep happened during case 4 (beep)


  //Objects
File dataFile;
  //Debouncing for switches
Bounce sw1,sw2;



//rolling average


void buzztone (int time, int frequency);

String fname="datalog.csv";
volatile bool isSetUp=false; //prob being read at the same time by both cores
void setup() {
  
  // put your setup code here, to run once:
  
  //Serial.setRX(1);
  //Serial.setTX(0);
  
  Serial.begin(115200);
delay(6000);
pinMode(LED_BUILTIN,OUTPUT);
      digitalWrite(LED_BUILTIN,HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN,LOW);
  // Ensure the SPI pinout the SD card is connected to is configured properly
  SPI.setRX(SPI_RX);
  SPI.setTX(SPI_TX);
  SPI.setSCK(SPI_SCLK);

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
}
/*
void setup1(){
while(!isSetUp){}//the cores arent sharing

//Serial.println("amogus");
}*/
File dataFile = SD.open(fname, FILE_WRITE);

void loop() {
  if (dataFile) {
    switch (state){
    case 0://mower lite solution
      String dataString = (String)millis() + ',' +
                          (String)-000.00 + ',' +
                          (String)-000.00  + ',' +
                          (String)-000.00 + ',' +
                          (String)-000.00 + ',' +
                          (String)-000.00 + ',' +
                          (String)-000.00 + ',' +
                          (String)-0000.00;
      dataFile.println(dataString);
      break;

    case 1://uhhh interesting solution
      dataFile.println(millis());dataFile.print(',');
      dataFile.print(-000.00);dataFile.print(',');
      dataFile.print(-000.00);dataFile.print(',');
      dataFile.print(-000.00);dataFile.print(',');
      dataFile.print(-000.00);dataFile.print(',');
      dataFile.print(-000.00);dataFile.print(',');
      dataFile.print(-000.00);dataFile.print(',');
      dataFile.print(0000.00);dataFile.print(',');
      break;

  
    // if the file is available, write to it:
 
      
  }
  else {
    Serial.println(fname);
  }
  }
  // print to the serial port too:
    
}

void buzztone (int time,int frequency = 1000) { //default frequency = 1000 Hz
  tone(BUZZ_PIN,frequency,time);
}
