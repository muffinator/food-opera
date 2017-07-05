#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "nrf.h"
#include <SPI.h>
#include <ADCTouch.h>
#include "FastLED.h"

//////////////////////////
//-------CONFIG---------//
//////////////////////////

#define VERSION 1
#define CHANNEL 2
#define BOARD 0
#define ENABLE_DRINKING 0
#define ENABLE_WAITER 0

//////////////////////////
//-------the rest-------//
//////////////////////////

FASTLED_USING_NAMESPACE
#define DATA_PIN    A5
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    1
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define M 10


//    OLD
#define ssp  6 //CSN
#define CE  5 //CE
#ifdef VERSION
//    NEW
#define ssp 17 //CSN
#define CE 11 //CE
#endif

Adafruit_BNO055 bno = Adafruit_BNO055();

int inbyte = 0;
uint8_t test[32];
uint8_t rxarray[13];
uint8_t bytetoss[4];
char bnoHere = 1;
int ref0, ref1, ref2, ref3;
int val0, val1, val2, val3;
int del0, del1, del2, del3;
int count;
char drank=0;
char waiter=0;
char slow;
char r,g,b;
uint8_t rxaddr[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
int threshold = 200;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(A4, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A4, HIGH);
  digitalWrite(A2, LOW);
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  
  pinMode(ssp, OUTPUT);
  pinMode(CE, OUTPUT);
  pinMode(12, INPUT_PULLUP);
  digitalWrite(CE, LOW);
  delay(200);
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.endTransaction();
  ref0 = ADCTouch.read(A0, 500);    //create reference values to 
  ref1 = ADCTouch.read(A1, 500);    //account for the capacitance of the pad
  ref2 = ADCTouch.read(A2, 500);
  ref3 = ADCTouch.read(A3, 500);
  val0 = ref0;
  val1 = ref1;
  val2 = ref2;
  val3 = ref3;
 
  if(BOARD == 0){
    rxaddr[0] = 0xa5;
  }
  if(BOARD == 1){
    rxaddr[0] = 0x53;
  }
  if(BOARD == 2){
    rxaddr[0] = 0x22;
  }
  if(BOARD == 3){
    rxaddr[0] = 0x0F;
  }
  if(BOARD == 4){
    rxaddr[0] = 0x4D;
  }
  if(BOARD == 5){
    rxaddr[0] = 0xB7;
  }
  
  for(uint8_t i=0;i<32;i++){
    test[i]=i;
  }
  
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    bnoHere = 0;
  }

  delay(1000);

  if(bnoHere){
  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);
  }
  
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  nrfWrite(EN_AA,(1<<ENAA_P0));  // auto ack pipe0,1
  nrfWrite(EN_RXADDR,(1<<ERX_P0));  //enable data pipe 0,1
  nrfWrite(SETUP_AW,(3<<AW));  //5 bite address width
  nrfWrite(SETUP_RETR,((5)<<ARD)|(5<<ARC));  //default 250uS retx delay, 4x retx
  nrfSetRxAddr(rxaddr,5,0);
  nrfSetTxAddr(rxaddr,5);
  nrfRead(10,5);
  nrfRead(0x10,5);
  nrfWrite(5, CHANNEL); //channel 2
  nrfWrite(RF_SETUP,(1<<RF_DR_LOW)|(1<<RF_PWR_LOW)|(1<<RF_PWR_HIGH)); //low data rate, low power (0b100000)
  //nrfWrite(0x1c,0x01); //enable dynamic payload data pipe 0
  //nrfWrite(0x1d,0x06); //enable dynamic payload with ack
  nrfWrite(0, 0x0E); //power on, PTX,2-byte crc
  nrfRead(6, 1);
  nrfWrite(7, 0x10);
  nrfRead(0x00, 1);
  //nrfFillTx(test,32);
  nrfRead(0x17, 1);
  nrfWrite(0xe1, 0); //clear tx fifo
  nrfRead(0x17, 1);
  //nrfWrite(0xa0,0xaa);
  nrfRead(0x17, 1);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  if(bnoHere){
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  nrfSendArr(1,(int)(M*euler.x()),(int)(M*euler.y()),(int)(M*euler.z()),0);
  
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  nrfSendArr(2,(int)(M*accelerometer.x()),(int)(M*accelerometer.y()),(int)(M*accelerometer.z()),0);

  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  nrfSendArr(4,(int)(M*gyroscope.x()),(int)(M*gyroscope.y()),(int)(M*gyroscope.z()),0);


  imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  nrfSendArr(8,(int)(M*magnetometer.x()),(int)(M*magnetometer.y()),(int)(M*magnetometer.z()),0);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  nrfSendArr(16,(int)(system),(int)(gyro),(int)(accel),(int)(mag));
  
/*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  }
  
  ////////////////////////////////
  //--------touch stuff---------//
  ////////////////////////////////
  
    del0 = ADCTouch.read(A0,22)-val0;
    val0 = del0+val0; //add old val to delta to get new val
    del1 = ADCTouch.read(A1,22)-val1;
    val1 = del1+val1; //add old val to delta to get new val
    del2 = ADCTouch.read(A2,22)-val2;
    val2 = del2+val2; //add old val to delta to get new val
    del3 = ADCTouch.read(A3,22)-val3;
    val3 = del3+val3; //add old val to delta to get new val
    nrfSendArr(32,(int)(val0),(int)(val1),(int)(val2),(int)(val3));

  ///////////////////////////////
  //--------LED stuff----------//
  ///////////////////////////////
  
//pin 0 - watching for water to touch someone's lips.
  if(ENABLE_DRINKING){
    threshold = 200;
    if (del0 > threshold) {
      leds[0].r = 255;
      leds[0].g = 255;
      leds[0].b = 255;
      drank = 1;
    }
    if (del0 < -1*threshold) {
      drank = 2;
    }
    
    if(drank==2&&leds[0].r>0){
      leds[0].r = max(leds[0].r-4,0);
      leds[0].g = max(leds[0].g-4,0);
      leds[0].b = max(leds[0].b-4,0);
      if(leds[0].r==0){
        drank=0;
      }
    }
  }
//pin 2 - the waiter's release
  if(ENABLE_WAITER){
    threshold = 500;
    if(val2 > threshold){
      count++;
    }else{
      count = 0;
    }
    if(count > 200){
      waiter = 1;
    }
    if((waiter == 1) && (count==0)){
      leds[0].r = min(leds[0].r+4,255);
      leds[0].b = min(leds[0].b+8,255);
      if(leds[0].b >= 255){
        waiter = 2;
      }
    }
    if(waiter == 2){
      slow++;
      if(slow == 10){
        leds[0].r = max(leds[0].r-1,0);
        leds[0].b = max(leds[0].b-2,0);
        if(leds[0].b <= 0){
          waiter=0;
        }
        slow = 0;
      }
    }
  }
    FastLED.show();

// end LED stuff


    delay(10);
  if((nrfRead(7,1)&0x10)==0x10){
    nrfWrite(7,0x10);
    nrfWrite(FLUSH_TX,0);
    Serial.println("MAX RETX");
    digitalWrite(CE, HIGH);
    delay(1);
    digitalWrite(CE, LOW);
  }
//  nrfWrite(7, 0x20);
//  nrfRead(7, 1);
//  nrfFillTx(test,4);
//  nrfRead(0x17, 1);
//  nrfRead(OBSERVE_TX,1);
//  digitalWrite(CE, HIGH);
//  delay(1);
//  digitalWrite(CE, LOW);
//  nrfRead(0x17, 1);
//  nrfRead(7, 1);
 delay(BNO055_SAMPLERATE_DELAY_MS/5);
}

void nrfFillTx(uint8_t data){
  digitalWrite(ssp, LOW);
  SPI.transfer(0xa0);
  SPI.transfer(data);
  digitalWrite(ssp, HIGH);
}

void nrfFillTx(uint8_t *data, uint8_t numBytes){
  digitalWrite(ssp, LOW);
  SPI.transfer(0xa0);
  for(char i=0;i<numBytes;i++){
    SPI.transfer(data[i]);
  }
  digitalWrite(ssp, HIGH);
}

void nrfSetRxAddr(uint8_t *data, uint8_t numBytes,uint8_t chan){
  digitalWrite(ssp, LOW);
  SPI.transfer((0x20|(10+chan)));
  for(char i=0;i<numBytes;i++){
    SPI.transfer(data[i]);
  }
  digitalWrite(ssp, HIGH);
}

void nrfSetTxAddr(uint8_t *data, uint8_t numBytes){
  digitalWrite(ssp, LOW);
  SPI.transfer(0x20|TX_ADDR);
  for(char i=0;i<numBytes;i++){
    SPI.transfer(data[i]);
  }
  digitalWrite(ssp, HIGH);
}

char nrfRead(int address, int numBytes) {
  int data[numBytes];
  digitalWrite(ssp, LOW);
  int STATUS = 0;
  STATUS = SPI.transfer(address);
  for (int i = 0; i < numBytes; i++) {
    data[i] = SPI.transfer(0);
  }
  digitalWrite(ssp, HIGH);
  Serial.print("status: ");
  Serial.println(STATUS, BIN);
  for (int i = 0; i < numBytes; i++) {
    Serial.print("reg ");
    Serial.print(address);
    Serial.print(" - data ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(data[i], BIN);
  }
  return STATUS;
}

void nrfWrite(char address, unsigned char data) {
  digitalWrite(ssp, LOW);
  int STATUS = 0;
  STATUS = SPI.transfer(0x20 | address);
  SPI.transfer(data);
  digitalWrite(ssp, HIGH);
}

void float2Byte(float val,byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}


void nrfSend(float datatx){
  nrfWrite(7, 0x20);
  float2Byte(datatx,bytetoss);
  nrfFillTx(bytetoss,4);
  digitalWrite(CE, HIGH);
  while(nrfRead(7,1)&0x20!=0x20){}
  nrfWrite(7,0x20);
  delay(10);
  digitalWrite(CE, LOW);
}

void nrfSendArr(int id, int x, int y, int z, int w)
{
  //nrfWrite(7, 0x20);
  uint8_t sendArray[10] = {(id&0xFF00)>>8, id&0xFF, (x&0xFF00)>>8, x&0xFF, (y&0xFF00)>>8, y&0xFF, (z&0xFF00)>>8, z&0xFF, (w&0xFF00)>>8, w&0xFF};
  nrfFillTx(sendArray, 10);
  digitalWrite(CE, HIGH);
  while(nrfRead(7,1)&0x20!=0x20){}
  nrfWrite(7,0x20);
  delay(9); 
  digitalWrite(CE, LOW);
}

