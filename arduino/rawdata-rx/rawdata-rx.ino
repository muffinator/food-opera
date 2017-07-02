#include <SPI.h>
#include <nrf.h>
#include <stdio.h>

//#define VERSION 0
#define CHANNEL 2
#define BOARD 0

//    OLD
#define ssp  6 //CSN
#define CE  5 //CE
#ifdef VERSION
//    NEW
#define ssp 17 //CSN
#define CE 11 //CE
#endif



uint8_t blah[32];
uint8_t datarx[32];
float refloat;
int pipe;
int tempx, tempy, tempz, tempw;
struct dof{
  int ox, oy, oz;
  int ax, ay, az;
  int gx, gy, gz;
  int mx, my, mz;
  int sc, gc, ac, mc;
  int t0, t1, t2, t3;
  uint8_t rxcount;
};

struct dof fork[6];

uint8_t rxaddr[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};

void setup() {
  while(!Serial);
  Serial.begin(115200);
  Serial.print("ORIENTATION: ");
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

  nrfWrite(EN_AA,(1<<ENAA_P0)|(1<<ENAA_P1));  // auto ack pipe0,1
  nrfWrite(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1));  //enable data pipe 0,1
  nrfWrite(SETUP_AW,(3<<AW));  //5 bite address width
  nrfWrite(SETUP_RETR,((5)<<ARD)|(5<<ARC));  //default 250uS retx delay, 3x retx
  
  //SET RX ADDRESSES
  rxaddr[0] = 0xA5;
  nrfSetRxAddr(rxaddr,5,0);
  
  rxaddr[0] = 0x53;
  nrfSetRxAddr(rxaddr,5,1);
  
  rxaddr[0] = 0x22;
  nrfSetRxAddr(rxaddr,1,2);

  rxaddr[0] = 0x0F;
  nrfSetRxAddr(rxaddr,1,3);

  rxaddr[0] = 0x4D;
  nrfSetRxAddr(rxaddr,1,4);

  rxaddr[0] = 0xB7;
  nrfSetRxAddr(rxaddr,1,5);
  
  nrfRead(0x0A,5);
  nrfRead(0x0B,5);
  nrfRead(0x0C,1);
  nrfRead(0x0D,1);
  nrfRead(0x0E,1);
  nrfRead(0x0F,1);
  
  //SET CHANNEL
  nrfWrite(RF_CH,CHANNEL); 
  nrfWrite(RF_SETUP,(1<<RF_DR_LOW)|(1<<RF_PWR_LOW)|(1<<RF_PWR_HIGH)); //low data rate, low power (0b100000)
  //nrfWrite(DYNPD,(1<<DPL_P0)); //enable dynamic payload data pipe 0
  //nrfWrite(FEATURE,(1<<EN_DPL)|(1<<EN_ACK_PAY)); //enable dynamic payload with ack
  
  nrfWrite(NRF_CONFIG,(1<<EN_CRC)|(1<<CRCO)|(1<<PWR_UP)|(1<<PRIM_RX));  //power on, PRX, 2-byte CRC
  nrfWrite(NRF_STATUS,(1<<RX_DR));  //clear rx_dr interrupt flag
  nrfWrite(FLUSH_RX,0); //clear rx fifo
  nrfRead(RF_SETUP,1); 
  nrfWrite(7,(1<<RX_DR));
  
  //SET DATA PACKET LENGTH
  nrfWrite(RX_PW_P0,10);
  nrfWrite(RX_PW_P1,10);
  nrfWrite(RX_PW_P2,10);
  nrfWrite(RX_PW_P3,10);
  nrfWrite(RX_PW_P4,10);
  nrfWrite(RX_PW_P5,10);
  
  nrfRead(FIFO_STATUS,1);
  nrfRead(NRF_CONFIG,(1<<PRIM_RX));
  digitalWrite(CE, HIGH);
  
}

void loop() {
  
  //delay(10);
  if(((nrfReadq(NRF_STATUS,1)[0]&(1<<RX_DR))==(1<<RX_DR))||
     ((nrfReadq(FIFO_STATUS,1)[0]&0x02)==0x02) ||
     ((nrfReadq(FIFO_STATUS,1)[0]&0x01)==0x01)){
       
      if(((nrfReadq(FIFO_STATUS,1)[0]&0x02)==0x02)){
        Serial.println("FIFO FUL");
        //Serial.println(a);
        nrfWrite(FLUSH_RX,0);
      }    
    
    uint8_t a = nrfReadq(R_RX_PL_WID,1)[0];
    //Serial.print(a);
    //Serial.println(a);
    if(a > 32) {  
      nrfWrite(FLUSH_RX,0);
    }else{
    //Serial.print(a);
    //delay(10);
    pipe = (((nrfReadq(NRF_STATUS,1)[0])&0x0f)>>1);
    nrfReadq(R_RX_PAYLOAD,a);
    }
    
    tempx = (datarx[2]<<8)+datarx[3];
    tempy = (datarx[4]<<8)+datarx[5];
    tempz = (datarx[6]<<8)+datarx[7];
    tempw = (datarx[8]<<8)+datarx[9];
    
    if(datarx[1]<=32){
      if(datarx[1]==2){ 
        fork[pipe].ax = tempx;
        fork[pipe].ay = tempy;
        fork[pipe].az = tempz;  
        if((fork[pipe].rxcount&2)==2){
          fork[pipe].rxcount = 2;
        }else{
          fork[pipe].rxcount |= 2;
        }
      }
      if(datarx[1]==4){
         
        fork[pipe].gx = tempx;
        fork[pipe].gy = tempy;
        fork[pipe].gz = tempz;
        if((fork[pipe].rxcount&4)==4){
          fork[pipe].rxcount = 4;
        }else{
          fork[pipe].rxcount |= 4;
        }   
      }
      if(datarx[1]==8){
        
        fork[pipe].mx = tempx;
        fork[pipe].my = tempy;
        fork[pipe].mz = tempz;
        if((fork[pipe].rxcount&8)==8){
          fork[pipe].rxcount = 8;
        }else{
          fork[pipe].rxcount |= 8;
        }   
      }
      if(datarx[1]==1){ 
        fork[pipe].ox = tempx;
        fork[pipe].oy = tempy;
        fork[pipe].oz = tempz;
        if((fork[pipe].rxcount&1)==1){
          fork[pipe].rxcount = 1;
        }else{
          fork[pipe].rxcount |= 1;
        }   
      }
      if(datarx[1]==16){
        fork[pipe].sc = tempx;
        fork[pipe].gc = tempy;
        fork[pipe].ac = tempz;
        fork[pipe].mc = tempw;
        if((fork[pipe].rxcount&16)==16){
          fork[pipe].rxcount = 16;
        }else{
          fork[pipe].rxcount |= 16;
        }       
      }
      if(datarx[1]==32){
        fork[pipe].t0 = tempx;
        fork[pipe].t1 = tempy;
        fork[pipe].t2 = tempz;
        fork[pipe].t3 = tempw;
        if((fork[pipe].rxcount&32)==32){
          fork[pipe].rxcount = 32;
        }else{
          fork[pipe].rxcount |= 32;
        }       
      }
    }
    //Serial.println(rxcount);
    if((fork[pipe].rxcount&0x1f) == 31){

      Serial.print("ORIENTATION: ");
      Serial.print(fork[pipe].ox, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].oy, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].oz, DEC);
      Serial.print("/");
      Serial.print("ACCELEROMETER:");
      Serial.print(fork[pipe].ax, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].ay, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].az, DEC);
      Serial.print("/");
      Serial.print("GYROSCOPE: ");
      Serial.print(fork[pipe].gx, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].gy, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].gz, DEC);
      Serial.print("/");
      Serial.print("MAGNETOMETER: "); 
      Serial.print(fork[pipe].mx, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].my, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].mz, DEC);
      Serial.print("/");
      Serial.print("CALIBRATION:");
      Serial.print(fork[pipe].sc, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].gc, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].ac, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].mc, DEC);
      Serial.println("/");
    }
    if((fork[pipe].rxcount&0x20) == 32){
      Serial.print("PIPE: ");
      Serial.print(pipe, DEC);
      Serial.print("TOUCH: ");
      Serial.print(fork[pipe].t0, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].t1, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].t2, DEC);
      Serial.print(":");
      Serial.print(fork[pipe].t3, DEC);
      Serial.println("/");
    }
    //delay(10);
    //byte2Float(datarx);
    //Serial.println(refloat);
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
    //nrfRead(0x17,1);
      if(((nrfReadq(FIFO_STATUS,1)[0]&0x02)==0x02)){
        Serial.println("FIFO FULL");
      }
      //Serial.println("fee fi fo fum");
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

void nrfSetRxAddr(uint8_t *data, uint8_t numBytes,uint8_t chan){
  digitalWrite(ssp, LOW);
  SPI.transfer(0x20|(10+chan));
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

