#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <mcp_can.h>

/*
Pin Location
   0   Serial Rx  (115200bps)
   1   serial Tx
   2        -NO USE
  -3   MCP2515 CAN INT
   4        -NO USE
  -5        -NO USE
  -6   LED Stick (NEO Pixel)
   7        -NO USE     
   8   Water/Methanol Empty Sensor
  -9        -NO USE 
  -10  MCP2515 SS
   11  MCP2515 MOSI
   12  MCP2515 MISO
   13  MCP2515 SCLK
   A0       -NO USE
   A1  W/M Flow                    *Flow Sencer
   A2       -NO USE
   A3  A/F sensor                  *Innovate A/F Sensor
   A4       -NO USE
   A5  knock sensor                *Knock Analyzer
   A6       -NO USE               
   A7       -NO USE 


server data
                                                                           
1:  A[0] oil temp                      0 → -40deg     A[0]=rxBuf[xx]-40;                                   data4
2:  A[1] coolant temp                  0 → -40deg     A[1]=rxBuf[3]-40;                                    data3
3:  A[2] engine speed                  0 → 0rpm       A[2]=int((rxBuf[3]*256+rxBuf[4])/4);                 data2 
4:  A[3] air pressure                  0 → 0kPa       A[3]=rxBuf[6]                                        data2 
5:  A[4] intake air temp               0 → -40deg     A[4]=rxBuf[5]-40                                     data3 
6:  A[5] mass air flow                 0 → 0g/s       A[5]=int((rxBuf[3]*256+rxBuf[4])/100);               data1     
7:  A[6] throttle                      0 → 0%         A[6]=int(rxBuf[6]/255*100);                          data1
    A[7] viecle speed                  0 → 0km/h      A[7]=rxBuf[7];                                       data3
    A[8] knock level                   0 → 0%         A[8]=analogread(A5)     　　                                 *Knock Analyzer
    A[9] A/F                           0 → 0          A[9]<-analogRead(A3) *x10　152 → 14.7 　　　　　　            *A/F meter 
    A[10] Water/Methanol Flow          0 → 0l/s       A[10]<-analogread(A1)     　　                                *flow meter
    A[11] Water/Methanol Level Check   0:empty         A[11]<-digitalRead(8)                                        *enmpty switch
    A[12]-[14] Waring Message
    A[15]-[16] Time Stamp 
*/


// Warning definition
#define oil_warning1      100
#define oil_warning2      110
#define coolant_warning1   90
#define coolant_warning2  100
#define knock_warning1     25
#define knock_warning2     50
#define rev1             6400
#define rev2             6800
#define rev3             7200

volatile int A[17] = { };
volatile unsigned long timeA;


// Neo Pixel ***********************************************************************************************
#define LED 6   
#define MAX_VAL 15// 0 to 255 for brightness
#define WAN_VAL 40// Warnig for brightness
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, LED, NEO_GRB + NEO_KHZ800);

void colorWipe(uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);strip.show(); }}

void rainbow() {
  uint16_t i, j;
  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));}
    strip.show(); }}

void rainbowCycle() {
  uint16_t i, j;
  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255)); }
    strip.show(); }}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) { return strip.Color((255 - WheelPos * 3) * MAX_VAL/255, 0, (WheelPos * 3) * MAX_VAL/255);}
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3 * MAX_VAL/255, (255 - WheelPos * 3) * MAX_VAL/255); }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3 * MAX_VAL/255, (255 - WheelPos * 3) * MAX_VAL/255 , 0); }


//CAN-BUS MCP2515 ******************************************************************************************
#define CAN0_INT 3                              // Set INT to pin 3
MCP_CAN CAN0(10);                               // Set CS to pin 10
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

unsigned long TM;  // TM:Trget Time
byte data1[8] = {0x03, 0x01, 0x10, 0x11, 0x00, 0x00, 0x00, 0x00}; // MAF          & Throttle position
byte data2[8] = {0x03, 0x01, 0x0C, 0x0B, 0x00, 0x00, 0x00, 0x00}; // Rev          & Air Pressure
byte data3[8] = {0x04, 0x01, 0x05, 0x0F, 0x0D, 0x00, 0x00, 0x00}; // Coolant Temp & Intake Air Temp   & Viecle Speed
byte data4[8] = {0x02, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}; // Oil Temp
byte data5[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Flow Control


void send_data() {
  char msg[100];
  A[15]=int(long(millis()/100000));
  A[16]=int(long(millis()%100000)/10);
  sprintf(msg,"S,%d,",A[0]);
  for( int h=1 ; h<17 ; h++ ) { sprintf( msg,"%s%d,",msg,A[h]); }
  sprintf( msg, "%s,E",msg );
  Serial.print(msg);
}


// Water/Methanol Empty Sensor   Pin 8 ********************************************************************
#define Level_Sensor 8 
void Level_Sensor_check() {
  A[11]=0;
  if ( digitalRead(Level_Sensor) == HIGH ) {
    if ( digitalRead(Level_Sensor) == HIGH ) {
      A[11]=1;
    }
  }
} 

// Water/Methanol flow            Pin A1 ******************************************************************
#define WMF A1
void WMF_calc() {
  int sc;
  sc=analogRead(WMF); //
  sc=analogRead(WMF); // scに0V-5.0Vが入る
  A[10]=map(sc,0,1023,0,500);
}

// Innovate AF  analog(0V:7.35 5.V:22.39)  Pin A3  ********************************************************
#define INNOVATE A3
void af_calc() {
  int sc;
  int cc;
  sc=analogRead(INNOVATE); //
  sc=analogRead(INNOVATE); // scに0V-5.0Vが入る
  cc=map(sc,0,1023,735,2239);cc=(cc-735)*29/20+735;
  A[9]=int(cc/10);
}

//Knock Analyzer                   Pin A5  ****************************************************************
#define knock_analyzer A5
void knock_calc() {
  A[8] = analogRead(knock_analyzer);
  A[8] = analogRead(knock_analyzer);
}




void setup(){
  Serial.begin(115200);     
  strip.begin();
  rainbowCycle();
  strip.show(); // Initialize all pixels to 'off'
  
  pinMode(CAN0_INT, INPUT);                        // Configuring pin for /INT input
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    //pass
  }
  CAN0.init_Mask(0,0,0x1F800000);                                       // Init first mask
  CAN0.init_Mask(1,0,0x1F800000);                                       // Init second mask
  for ( int i=0 ; i < 6 ; i++ ) {
    CAN0.init_Filt(i,0,0x1FA00000);                                     // Init filters
  }
  CAN0.setMode(MCP_NORMAL);                                             // Set operation mode to normal so the MCP2515 sends acks to received data.
 
  colorWipe(strip.Color(0, 0, 0)); // Black
  timeA=millis();
}



void loop() {
  while ( timeA > millis() ) {  }
  timeA=millis()+200;
  int k=0;
  while ( k < 10 ) {
    long timeB = millis();
    if ( k == 2 ) CAN0.sendMsgBuf( 0x7E0, 0, 8, data4);
    if ( k == 4 ) CAN0.sendMsgBuf( 0x7E0, 0, 8, data3);
    if ( k == 6 ) CAN0.sendMsgBuf( 0x7E0, 0, 8, data1);
    if ( k == 8 ) CAN0.sendMsgBuf( 0x7E0, 0, 8, data2);
    while ( timeB+20>millis() ) read_can();
    k++;
  }
  process_data();
}

void read_can() {
 while(!digitalRead(CAN0_INT)) {                        // If CAN0_INT pin is low, read receive buffer
      CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
      if ( rxId == 0x7E8 ) {
        if ( rxBuf[0]==0x06 && rxBuf[1]==0x41 ) {
          if ( rxBuf[2] == 0x10 ) {                                              // data1 mass air flow , Throttle
            A[5]=int((rxBuf[3]*256+rxBuf[4])/100);
            A[6]=int(rxBuf[6]*100/255);
          }
          if ( rxBuf[2] == 0x0C ) {                                              // data2 engine speed,air pressure
            A[2]=int((rxBuf[3]*256+rxBuf[4])/4);
            A[3]=int(rxBuf[6]);
          }  
        }
        if ( rxBuf[0]==0x07 && rxBuf[1]==0x41 && rxBuf[2]==0x05 ) {              // data3 coolant temp,intake air temp,viecle speed
            A[1]=int(rxBuf[3]-40);
            A[4]=int(rxBuf[5]-40);
            A[7]=int(rxBuf[7]);
        }
        if ( rxBuf[0]==0x10 && rxBuf[2]==0x61 ) { CAN0.sendMsgBuf( 0x7E0, 0, 8, data5); }  // Flow Control return
        if ( rxBuf[0]==0x24 ) {                                                   // data4 Oil Temp
            A[0]=int(rxBuf[4]-40);
        }
      }
    }
  }


void process_data() {
  af_calc();
  WMF_calc();
  Level_Sensor_check();
  if ( A[0] >= oil_warning2 ) { A[7] =2 ; } else { if ( A[0] >= oil_warning1 ) { A[12] =1 ; } else { A[12] =0 ;} }
  if ( A[1] >= coolant_warning2 ) { A[13] =2 ; } else { if ( A[1] >= coolant_warning1 ) { A[13] =1 ; } else { A[13] =0 ;} }
  if ( A[2] >= rev2 ) { A[14] =2 ; } else { if ( A[2] >= rev1 ) { A[14] =1 ; } else { A[14] =0 ;} }
  send_data();
  flush_LED();
}

void flush_LED() {
  for ( int h=0 ; h<8 ; h ++ ) {  strip.setPixelColor( h, strip.Color(0, 0, 0)); }
  if (A[12] ==2 ) {strip.setPixelColor(0, strip.Color(WAN_VAL, 0, 0));  }                  //1 : oil temp         orange -> red
    if (A[12] ==1 ) {strip.setPixelColor(0, strip.Color(MAX_VAL, MAX_VAL/2, 0));  }
  if (A[13] ==2 ) {strip.setPixelColor(1, strip.Color(WAN_VAL, 0, 0));  }                  //2 : coolant temp     orange -> red
    if (A[13] ==1 ) {strip.setPixelColor(1, strip.Color(MAX_VAL, MAX_VAL/2, 0));  }
  if (A[14] ==2 ) {strip.setPixelColor(2, strip.Color(WAN_VAL, 0, 0));  }                  //3 : knock level      orange -> red
    if (A[14] ==1 ) {strip.setPixelColor(2, strip.Color(MAX_VAL, MAX_VAL/2, 0));  }
  if (A[11]==1 ) {strip.setPixelColor(3, strip.Color(MAX_VAL/2, MAX_VAL/2 ,MAX_VAL/2));}   //4 : W/M level        white
  if (A[10] >= 125 ) {strip.setPixelColor(4, strip.Color(0, 0 , MAX_VAL/2 ));  }           //5 : W/M Flow         green  -> blue 
    if (A[10] >= 80  ) {strip.setPixelColor(4, strip.Color(0, MAX_VAL/2, 0 ));  }
  if (A[2]>=rev1) { strip.setPixelColor(5, strip.Color(MAX_VAL, MAX_VAL, 0));  }           //6 : rev1             yellow
  if (A[2]>=rev2) { strip.setPixelColor(6, strip.Color(MAX_VAL, MAX_VAL/2, 0));  }         //7 : rev2             orange
  if (A[2]>=rev3) { strip.setPixelColor(7, strip.Color(WAN_VAL, 0, 0));  }                 //8 : rev3             red

  strip.show();
}
