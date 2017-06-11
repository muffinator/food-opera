#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "nrf.h"
#include <SPI.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

const int ssp = 6; //CSN
const int CE = 5; //CE
int inbyte = 0;
uint8_t test[32];
uint8_t rxarray[13];
uint8_t bytetoss[4];


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(ssp, OUTPUT);
  pinMode(CE, OUTPUT);
  pinMode(12, INPUT_PULLUP);
  digitalWrite(CE, LOW);
  delay(200);
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.endTransaction();

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
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  nrfWrite(EN_AA,(1<<ENAA_P0));  // auto ack pipe0
  nrfWrite(EN_RXADDR,(1<<ERX_P0));  //enable data pipe 0
  nrfWrite(SETUP_AW,(3<<AW));  //5 bite address width
  nrfWrite(SETUP_RETR,((15)<<ARD)|(5<<ARC));  //default 250uS retx delay, 3x retx
  nrfWrite(5, 2); //channel 2
  nrfWrite(6, 0x20); //low data rate, low power (0b100000)
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
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

//  /* Display the floating point data */
//  Serial.print("ORIENTATION: ");
//  Serial.print("X: ");
//  Serial.print(euler.x());
//  Serial.print(" Y: ");
//  Serial.print(euler.y());
//  Serial.print(" Z: ");
//  Serial.print(euler.z());
//  Serial.print("\t\t");
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  nrfSendArr(0,(int)(10*euler.x()),(int)(10*euler.y()),(int)(10*euler.z()));
  //Serial.println("euler");
//  nrfSend(euler.x());
//  nrfSend(euler.y());
//  nrfSend(euler.z());

  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  nrfSendArr(1,(int)(10*accelerometer.x()),(int)(10*accelerometer.y()),(int)(10*accelerometer.z()));
  //Serial.println("accel");
//  nrfSend(accelerometer.x());
//  nrfSend(accelerometer.y());
//  nrfSend(accelerometer.z());

  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  nrfSendArr(2,(int)(10*gyroscope.x()),(int)(10*gyroscope.y()),(int)(10*gyroscope.z()));
  //Serial.println("gyro");
//  nrfSend(gyroscope.x());
//  nrfSend(gyroscope.y());
//  nrfSend(gyroscope.z());


  imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  nrfSendArr(3,(int)(10*magnetometer.x()),(int)(10*magnetometer.y()),(int)(10*magnetometer.z()));
  //Serial.println("mag");
//  nrfSend(magnetometer.x());
//  nrfSend(magnetometer.y());
//  nrfSend(magnetometer.z());

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  nrfSendArr(4,(int)(system),(int)(gyro),(int)(accel));
  //nrfSend(system);
  //nrfSend(gyro);
  //nrfSend(accel);
  //nrfSend(mag);
  //Serial.println("sys");
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

//  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//  Serial.print("ACCELEROMETER: ");
//  Serial.print("X: ");
//  Serial.print(accelerometer.x());
//  Serial.print(" Y: ");
//  Serial.print(accelerometer.y());
//  Serial.print(" Z: ");
//  Serial.print(accelerometer.z());
//  Serial.print("\t\t");
//
//  
//  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//  Serial.print("GYROSCOPE: ");
//  Serial.print("X: ");
//  Serial.print(gyroscope.x());
//  Serial.print(" Y: ");
//  Serial.print(gyroscope.y());
//  Serial.print(" Z: ");
//  Serial.print(gyroscope.z());
//  Serial.print("\t\t");
//  
//  imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
//  Serial.print("MAGNETOMETER: ");
//  Serial.print("X: ");
//  Serial.print(magnetometer.x());
//  Serial.print(" Y: ");
//  Serial.print(magnetometer.y());
//  Serial.print(" Z: ");
//  Serial.print(magnetometer.z());
//  Serial.print("\t\t");
//  
//  /* Display calibration status for each sensor. */
//  uint8_t system, gyro, accel, mag = 0;
//  bno.getCalibration(&system, &gyro, &accel, &mag);
//  Serial.print("CALIBRATION: Sys=");
//  Serial.print(system, DEC);
//  Serial.print(" Gyro=");
//  Serial.print(gyro, DEC);
//  Serial.print(" Accel=");
//  Serial.print(accel, DEC);
//  Serial.print(" Mag=");
//  Serial.println(mag, DEC);
delay(100);
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

void nrfSendArr(int id, int x, int y, int z)
{
  //nrfWrite(7, 0x20);
  uint8_t sendArray[8] = {(id&0xFF00)>>8, id&0xFF, (x&0xFF00)>>8, x&0xFF, (y&0xFF00)>>8, y&0xFF, (z&0xFF00)>>8, z&0xFF};
  nrfFillTx(sendArray, 8);
  digitalWrite(CE, HIGH);
  while(nrfRead(7,1)&0x20!=0x20){}
  nrfWrite(7,0x20);
  delay(8); 
  digitalWrite(CE, LOW);
}

