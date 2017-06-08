#include <SPI.h>
#include <nrf.h>

// set pin 10 as the slave select for the digital pot:
const int ssp = 5; //CSN
const int CE = 6; //CE
uint8_t blah[32];
uint8_t datarx[32];
float refloat;
int tempx;
int tempy;
int tempz;


void setup() {
  datarx[0]=0;
  for(uint8_t i=0;i<32;i++){
    blah[i]=i;
    datarx[i]=0;
  }
  // set the slaveSelectPin as an output:
  pinMode(ssp, OUTPUT);
  pinMode(CE, OUTPUT);
  pinMode(12, INPUT_PULLUP);
  digitalWrite(CE, LOW);
  // initialize SPI:
  delay(200);
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.endTransaction();
  Serial.begin(115200);

  nrfWrite(EN_AA,(1<<ENAA_P0));  // auto ack pipe0
  nrfWrite(EN_RXADDR,(1<<ERX_P0));  //enable data pipe 0
  nrfWrite(SETUP_AW,(3<<AW));  //5 bite address width
  nrfWrite(SETUP_RETR,((15)<<ARD)|(5<<ARC));  //default 250uS retx delay, 3x retx
  nrfWrite(RF_CH,2);  //channel 2
  nrfWrite(RF_SETUP,(1<<RF_DR_LOW)|(0<<RF_PWR)); //low data rate, low power (0b100000)
  //nrfWrite(DYNPD,(1<<DPL_P0)); //enable dynamic payload data pipe 0
  //nrfWrite(FEATURE,(1<<EN_DPL)|(1<<EN_ACK_PAY)); //enable dynamic payload with ack
  
  nrfWrite(NRF_CONFIG,(1<<EN_CRC)|(1<<CRCO)|(1<<PWR_UP)|(1<<PRIM_RX));  //power on, PRX, 2-byte CRC
  nrfWrite(NRF_STATUS,(1<<RX_DR));  //clear rx_dr interrupt flag
  nrfWrite(FLUSH_RX,0); //clear rx fifo
  nrfRead(6,1); 
  nrfWrite(7,0x40);
  nrfWrite(0x11,8);
  nrfRead(0x17,1);
  nrfRead(0,1);
  digitalWrite(CE, HIGH);
}

void loop() {
  delay(10);
  if((nrfReadq(NRF_STATUS,1)[0]&(1<<RX_DR))==(1<<RX_DR)||
     ((nrfReadq(FIFO_STATUS,1)[1]&0x01)==1)){
    uint8_t a = nrfReadq(R_RX_PL_WID,1)[0];
    if(a > 32) {  
      nrfWrite(FLUSH_RX,0);
    }
    
    //delay(10);
    nrfReadq(0x61,a);
    tempx = (datarx[2]<<8)+datarx[3];
    tempy = (datarx[4]<<8)+datarx[5];
    tempz = (datarx[6]<<8)+datarx[7];
    if(datarx[1]<4){
      if(datarx[1]==1){
        Serial.print("ACCELEROMETER:");   
      }
      if(datarx[1]==2){
        Serial.print("GYROSCOPE: ");   
      }
      if(datarx[1]==3){
        Serial.print("MAGNETOMETER: ");   
      }
      if(datarx[1]==0){
        Serial.print("ORIENTATION: ");   
      }
      //Serial.print("X: ");
      Serial.print(tempx);
      Serial.print(":");
      Serial.print(tempy);
      Serial.print(":");
      Serial.print(tempz);
      Serial.print("/");
    }else{
      Serial.print("CALIBRATION:");
      Serial.print(tempx, DEC);
      Serial.print(":");
      Serial.print(tempy, DEC);
      Serial.print(":");
      Serial.print(tempz, DEC);
      //Serial.print(" Mag=");
      //Serial.println(mag, DEC);
      Serial.println("/");
    }
    //delay(10);
    //byte2Float(datarx);
    //Serial.println(refloat);
    for(char i=0;i<a;i++){
      //Serial.print(datarx[i],HEX);
    }
    //Serial.print("\n");
    //Serial.println(10);
    //Serial.println(dat[10],HEX);
    //Serial.println((nrfReadq(0x17,1)[1]&0x01), HEX);
    nrfWrite(7,0x40);
    /*while((nrfReadq(0x17,1)[1]&0x01)!=1){
      //delay(10);
      uint8_t a = nrfReadq(R_RX_PL_WID,1)[1];
      unsigned char *dat = nrfReadq(0x61,32);
      //delay(10);
      for(char i=0;i<a;i++){
        Serial.print(blah[i],HEX);
      }
      Serial.println(blah[a],HEX);
    }*/
    //nrfRead(7,1);
  }
}

uint8_t *nrfReadq(unsigned char address, unsigned char numBy){
  digitalWrite(ssp, LOW);
  datarx[numBy] = SPI.transfer(address);
  for(int i=0; i<(numBy);i++){
    datarx[i]=SPI.transfer(0);
  }
  digitalWrite(ssp, HIGH);
  return datarx;
}

void nrfWriteq(unsigned char address, unsigned char numBytes, unsigned char *data){

  
}

char nrfRead(int address, int numBytes) {
  int data[numBytes];
  digitalWrite(ssp, LOW);
  int STATUS = 0;
  STATUS = SPI.transfer(address);
  for(int i=0; i<numBytes;i++){
    data[i]=SPI.transfer(0);
  }
  digitalWrite(ssp, HIGH);
  Serial.print("status: ");
  Serial.println(STATUS,BIN);
  for(int i=0; i<numBytes; i++){
    Serial.print("reg ");
    Serial.print(address);
    Serial.print(" - data ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(data[i],BIN);
  }
  return STATUS;
}

void nrfWrite(char address, unsigned char data) {
  digitalWrite(ssp, LOW);
  int STATUS = 0;
  STATUS = SPI.transfer(0x20|address);
  SPI.transfer(data);
  digitalWrite(ssp, HIGH);
}

void byte2Float(byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  memcpy(u.temp_array, bytes_array,  sizeof u.temp_array);
  // Overite bytes of union with float variable
  refloat = u.float_variable;
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

