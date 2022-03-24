#include <stdio.h>
#include <Adafruit_GFX.h>
#include <UTFTGLUE.h>
#include <SD.h>
#include <SPI.h>

//Status ***********************************************************************************************
volatile int SD_ERR = 0 ;  // 0 NO ERROR　           1 ERROR
volatile int bright=120;   // LCD輝度
volatile int A[3][17]= { } ;  //data　[0][] present　[1][] peak [2][] previous

/*
Pin Location

0-1        Serial Bourate  56700bps 
2-9,A0-A4  LCD Shield
10-13      SD Card         SSpin 10
A5         Push Switch    ( Mode Switch : Monitor mode <-> Logging mode )


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


//3.5' TFT  320*480 ID= 0x9481  ************************************************************************

UTFTGLUE tft(0x9481,A2,A1,A3,A4,A0);

//color definition
#define blue     2
#define green   20
#define red    200
#define orange 210
#define yellow 220
#define white  222
#define purple 202
#define aqua    22
#define black    0 

//TFT Control
void Cursor2(int x,int y,int c) {                                             // FontSize 2 ->  38*8 char
    tft.setTextSize(2);
    tft.setCursor(x*12+12, y*36+8);
    tft.setColor(255-int(c/100)*bright,255-(int(c/10)-int(c/100)*10)*bright,255-int(c-int(c/10)*10)*bright);
}

void Cursor3(int x,int y,int c) {                                             // FontSize 3 ->  25*8 char
    tft.setTextSize(3);
    tft.setCursor(x*12+8, y*36+4);
    tft.setColor(255-int(c/100)*bright,255-(int(c/10)-int(c/100)*10)*bright,255-int(c-int(c/10)*10)*bright);
}

void Line( int x1 , int y1 , int x2 , int y2 , int c ) {                      // draw lines
    tft.setColor(255-int(c/100)*bright,255-(int(c/10)-int(c/100)*10)*bright,255-int(c-int(c/10)*10)*bright);
    tft.drawLine( x1 , y1 , x2 , y2 );
}

void tft_init(int c) {                                                        // TFT initial screen
    char d;
    tft.fillScreen(0xFFFF);
    for ( int i=0 ; i<8 ; i++) {
      tft.setColor(255-int(c/100)*bright*(9-i)/9,255-(int(c/10)-int(c/100)*10)*bright*(9-i)/9,255-int(c-int(c/10)*10)*bright*(9-i)/9);
      tft.drawLine(0, 3*i+0, 479-(3*i+0)*10, 3*i+0);
      tft.drawLine(0, 3*i+1, 479-(3*i+1)*10, 3*i+1);
      tft.drawLine(0, 3*i+2, 479-(3*i+2)*10, 3*i+2);
      tft.drawLine(0, 319-(3*i+0), 479, 319-(3*i+0));
      tft.drawLine(0, 319-(3*i+1), 479, 319-(3*i+1));
      tft.drawLine(0, 319-(3*i+2), 479, 319-(3*i+2));
    }

//  Cursor2(0,0,aqua); tft.println(  "01234567890123456789012345678901234567"); // size3で12から5文字　27から5文字
    Cursor2(0,1,green);tft.println(F(" Oil Temp :          deg (           )"));
    Cursor2(0,2,green);tft.println(F(" Coolant  :          deg (           )"));
    Cursor2(0,3,green);tft.println(F(" Eg Speed :          rpm (           )"));
    Cursor2(0,4,green);tft.println(F(" Air Pres :          bar (           )"));
    Cursor2(0,5,green);tft.println(F(" Air Temp :          deg (           )"));
    Cursor2(0,6,green);tft.println(F(" Air Flow :          g/s (           )"));
    Cursor2(0,7,green);tft.println(F(" Throttle :           %  (           )"));
    while(Serial.available()) { d=Serial.read(); }
}


//SD CARD ***********************************************************************************************
int file_number;
char file_name[15];
File myFile;
File root;

void SD_check() {
    if (!SD.begin(10)) {
      SD_ERR = 1;
      SD_fail();
    } else {
      root = SD.open("/");
      int j=0;
      while (1) {
        File entry =  root.openNextFile();
        if (! entry) {  break; }
        sprintf(file_name,"%s",entry.name());
        if(file_name[0]=='L' && file_name[1]=='O' && file_name[2]=='G'){
          j=(file_name[3]-48)*100+(file_name[4]-48)*10+(file_name[5]-48);
          if ( j > file_number  ) { file_number = j ; }
      }
      entry.close();
    }
    root.close();
  }
}

void SD_fail() {
  tft.fillScreen(0xFFFF);
  Cursor2(0,3,red);tft.println("       SD-Card Failed !");
  Cursor2(0,4,red);tft.println("         Check Card ");
  delay(1000);
}

//*********************************************************************************************************

void setup()
{  
  pinMode(A5,INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial) {}
  tft.reset();
  tft.begin(0x9481);
  tft.setRotation(3);
  tft.setBackColor(0xFFFF);
  SD_check();
  clear_data();
}
 
void loop()
{
  clear_data();
  tft_init(green);
  Cursor2(0,0,green)    ;tft.println("Monitoring Mode");

  while( digitalRead(A5) == HIGH ) { //digitalRead(A5) == HIGH
    processing_data();                                    // Monitoring Mode **************************************************************************************
  }

  clear_data();
  tft.fillScreen(0x0000);
  delay(200);
    
  if ( SD_ERR == 0 ) {
    file_number++;
    sprintf(file_name,"LOG%03d.TXT",file_number);
    myFile = SD.open(file_name, FILE_WRITE);
    if (myFile) myFile.println(F("oil temp,coolant temp,air temp,engine speed,air press,air temp, air flow,throttle,viecle speed,knock level,A/F,W/M flow,W/M level,alert1,alert2,alert3,time"));
    myFile.close();
    tft_init(blue);
    Cursor2(0,0,blue)    ;tft.println("Logging Mode");
    
    while( digitalRead(A5) == HIGH ) {
      recording_data();                                    // Logging Mode **************************************************************************************
    }
    tft.fillScreen(0x0000);
    delay(200);
  } else {
    SD_fail();
  }
}




char read_data() {
    while(1) {
      if ( Serial.available()>0 ) { break; }
    }
    return Serial.read();
}

void clear_data() {
  for ( int i = 0 ; i < 15 ; i++ ) { A[1][i] = 0; A[2][i]= 0; } //  previous data , peak data initialize
}

// data processing ( monitor mode ) **************************************************************************************

int processing_data() {
  int h=-1;
  char c;
  char str[10];
  char data[100];
  long TimeA;
  TimeA = millis();
  while ( TimeA+200 > millis() ){
    if ( Serial.available() > 0 ) {
      h=0;
      break;
    }
  }
  if ( h == -1 ) return 0;
  while(1) {
    c=read_data();
    if ( c == 'S' ) { c=read_data();break; }
  }
  while(1) {
    c=read_data();
    if ( c == 'E' ) { break; }
    data[h]=c;
    h++;
  }
  data[h] = '\0';
  if ( h ==0 ) { return 0; }
  A[0][0] = atoi(strtok(data,","));
  for ( int i = 1 ; i < 17 ; i++ ) { A[0][i] = atoi(strtok(NULL,",")); }

  for ( int i = 0 ; i < 3 ; i++ ) {
    if ( A[0][i] == 0 ) { Cursor3(13,i+1,green)    ;tft.println("    "); } else {
      if (  A[2][i] != A[0][i] ) {
        sprintf ( str , "%4d" , A[0][i] );
        switch ( A[0][i+12] ) { 
          case 2: Cursor3(13,i+1,red);  break;
          case 1: Cursor3(13,i+1,yellow); break;
          default: Cursor3(13,i+1,white); break;
        }
        tft.println(str);
      
        if (  A[0][i] > A[1][i] )  { 
          A[1][i] = A[0][i] ;
          switch ( A[0][i+12] ) { 
            case 2: Cursor3(29,i+1,red); break;
            case 1: Cursor3(29,i+1,yellow); break;
            default: Cursor3(29,i+1,white); break;
          }
        tft.println(str);    
        }
      }
    }
  }

  for ( int i =3 ; i<7 ; i++ ) { 
    if ( A[0][i] == 0 ) { Cursor3(13,i+1,green)    ;tft.println("    "); } else {
      if (  A[2][i] != A[0][i] ) {
        sprintf ( str , "%4d" , A[0][i] );
        Cursor3(13,i+1,white);tft.println(str);
        if (  A[0][i] > A[1][i] )  { 
          A[1][i] = A[0][i];
          sprintf ( str , "%4d" , A[0][i] );
          Cursor3(29,i+1,white);tft.println(str);        
        }
      }
    }
  }

  for ( int i = 0 ; i < 7 ; i++ ) { A[2][i] = A[0][i]; }
  
  return 1;
}


// data recording ( logging mode ) **************************************************************************************

int recording_data() {
  int h=0;
  char c;
  char str[10];
  char data[100];
  char msg[100];
  long TimeA;
  TimeA = millis();
  while ( TimeA+200 > millis() ){
    if ( Serial.available() > 0 ) {
      h=0;
      break;
    }
  }
  if ( h == -1 ) return 0;
  while(1) {
    c=read_data();
    if ( c == 'S' ) { c=read_data();break; }
  }
  while(1) {
    c=read_data();
    if ( c == 'E' ) { break; }
    data[h]=c;
    h++;
  }
  data[h] = '\0';
  if ( h ==0 ) { return 0; }
  for ( int i = 0 ; i < 100 ; i++ ) msg[i]=data[i];
  A[0][0] = atoi(strtok(data,","));
  for ( int i = 1 ; i < 17 ; i++ ) { A[0][i] = atoi(strtok(NULL,",")); }
  
  for ( int i =0 ; i<7 ; i++ ) { 
    if ( A[0][i] == 0 ) { 
      Cursor3(13,i+1,green)    ;tft.println("    ");
    } else {
      if (  A[2][i] != A[0][i] ) {
        sprintf ( str , "%4d" , A[0][i] );
        Cursor3(13,i+1,white);
        tft.println(str);
      }
    }
  }

  for ( int i = 0 ; i < 10 ; i++ ) { A[2][i] = A[0][i]; }
  
  myFile = SD.open(file_name, FILE_WRITE);
  if (myFile) myFile.println(msg);
  myFile.close();
  
  return 1;
}
