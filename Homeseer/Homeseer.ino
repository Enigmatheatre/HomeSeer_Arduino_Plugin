#include <EEPROM.h>


/********************************************************
 *Arduino to Homeseer 3 Plugin writen by Enigma Theatre.*
 * V1.0.0.40                                            *
 *                                                      *
 *******Do not Change any values below*******************
 */


//Global Variables
#define ISIP 0
const byte BoardAdd = 1;
byte Byte1,Byte2,Byte3;
int Byte4,Byte5;
char* Version = "1.0.0.40";
bool IsConnected = false;
void(* resetFunc) (void) = 0; 
byte EEpromVersion = EEPROM.read(250);


//******************************Ethernet Setup*****************************
#if ISIP == 1

#include <SPI.h>       
#include <Ethernet.h>
#include <EthernetUdp.h> 

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192,168,0,145);     //IP entered in HS config.
const unsigned int localPort = 9000;      //port entered in HS config.
IPAddress HomeseerIP(192,168,0,20); //Homeseer IP address
IPAddress ServerIP(EEPROM.read(2),EEPROM.read(3),EEPROM.read(4),EEPROM.read(5));

char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

EthernetUDP Udp;
const unsigned int ServerPort = 8888;     // port to Send To

void UDPCheck(){
  byte packetSize = Udp.parsePacket();
  if(packetSize)
  {
    IPAddress remote = Udp.remoteIP();
    Byte1 =Udp.parseInt();
    Udp.read(); 
    Byte2 =Udp.read(); 
    Byte3 =Udp.parseInt();
    Byte4 =Udp.parseInt();
    Byte5 =Udp.parseInt();
    DataEvent();
  }
}
#endif




//********************************Input Setup*******************************
byte InPinArray[30] = { 0 };
byte Debounce = 30;
byte count = 0; 
byte NoOfInPins = 0; 
int InStateArray  [(sizeof(InPinArray) / sizeof(InPinArray[0]))];
unsigned long PrevDebounce [(sizeof(InPinArray) / sizeof(InPinArray[0]))];

void InputCheck(){
  byte pinread;
  for (count=0;count<NoOfInPins;count++) {
    if(millis() - PrevDebounce[count] > Debounce){
      pinread = (digitalRead (InPinArray[count]));
      if (InStateArray[count] != pinread){
        InStateArray[count] = pinread;
        PrevDebounce[count] = millis();
        Send(BoardAdd); 
        Send(" I ");
        Send(count+1); 
        Send(" ");  
        Send(pinread); 
        Send();
      }
    }
  }
}

//*******************************Analogue Setup****************************
byte  AnalogPinArray[15] = {0};
int AnalogueDelay[(sizeof(AnalogPinArray) / sizeof(AnalogPinArray[0]))] = {0};
word AnalogueInvert = 0; 
byte NoOfAnalogPins =0;
int AnalogStateArray[(sizeof(AnalogPinArray) / sizeof(AnalogPinArray[0]))];
unsigned long PrevAnalogeMillis[sizeof(AnalogPinArray) / sizeof(AnalogPinArray[0])]; 

  void  AnalogueCheck(){
    int pinread;
    for (count=0;count<NoOfAnalogPins;count++) {
      if(millis() - PrevAnalogeMillis[count] > AnalogueDelay[count]) {
        PrevAnalogeMillis[count] = millis(); 
        pinread = analogRead(AnalogPinArray[count]);
        if (AnalogStateArray[count] != pinread){
          AnalogStateArray[count] = pinread;
          Send(BoardAdd); 
          Send(" A ");
          Send(count+1); 
          Send(" ");  
          if (bitRead(AnalogueInvert,count) == 1){
            Send(map(AnalogStateArray[count], 0, 1023, 1023, 0)); 
            Send();
          }
          else{
            Send(AnalogStateArray[count]);
            Send();
          }
        }
      }
    }
  }


//*****************************PWM Setup****************************
byte PwmPinArray[25] = {0};
byte NoOfPwmPins = 0;
int PwmStateArray[(sizeof(PwmPinArray) / sizeof(PwmPinArray[0]))];
int PwmFadeTime[(sizeof(PwmPinArray) / sizeof(PwmPinArray[0]))];
int fadeTarget[(sizeof(PwmPinArray) / sizeof(PwmPinArray[0]))];
int fadeValueTweened[(sizeof(PwmPinArray) / sizeof(PwmPinArray[0]))];
int fadeValue[(sizeof(PwmPinArray) / sizeof(PwmPinArray[0]))];
unsigned long fadeTimerLast[(sizeof(PwmPinArray) / sizeof(PwmPinArray[0]))];

void PWMCheck(){
  for (count=0;count<NoOfPwmPins;count++) { 
    if ( abs(millis() - fadeTimerLast[count]) >= 20) {
      fadeTimerLast[count] = millis();
      float fadeStep = (float(20) / (PwmFadeTime[count])) * 254;
      if(PwmStateArray[count] > fadeValue[count]){
        fadeValue[count] = fadeValue[count] + fadeStep; 
      }
      if(PwmStateArray[count] < fadeValue[count]){
        fadeValue[count] = fadeValue[count] - fadeStep; 
      }
      if(fadeValue[count] == 0 ){
        analogWrite(PwmPinArray[count], PwmStateArray[count]);
      }
      fadeValue[count] = constrain(fadeValue[count], 0, 254);
      fadeValueTweened[count] = Quad_easeInOut(fadeValue[count], 0, 254);
      if(fadeValue[count] > 0 ){
        analogWrite(PwmPinArray[count], fadeValueTweened[count]);
      }
    }
  }
}

float Quad_easeInOut(float t, float fixedScaleStart, float fixedScaleEnd){
  //   float b = 0, c = 1, d = 1;
  float b = fixedScaleStart;
  float c = fixedScaleEnd - fixedScaleStart;
  float d = fixedScaleEnd;
  if ((t/=d/2) < 1) return c/2*t*t + b;
  return -c/2 * ((--t)*(t-2) - 1) + b;    
}


//*****************************Servo Setup***************************
#include <Servo.h> 
byte ServoPinArray[8] = {0};
byte NoOfServos = 0;
Servo myservo[(sizeof(ServoPinArray) / sizeof(ServoPinArray[0]) )];
int ServoPosArray[(sizeof(ServoPinArray) / sizeof(ServoPinArray[0]))];
int ServoOldPosArray[(sizeof(ServoPinArray) / sizeof(ServoPinArray[0]))];
int ServoSpeedArray[(sizeof(ServoPinArray) / sizeof(ServoPinArray[0]))];

void ServoCheck(){
  for (count=0;count<NoOfServos;count++) { 
    if (ServoPosArray[count] != ServoOldPosArray[count]){
      myservo[count].write(ServoPosArray[count]);
      ServoOldPosArray[count] = ServoPosArray[count];
    }
  }
}


//***************************One Wire Setup**************************
#include <OneWire.h>
#include <DallasTemperature.h>
byte OneWirePin = EEPROM.read(1);
byte TEMPERATURE_PRECISION = EEPROM.read(6);
OneWire oneWire(OneWirePin);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
int OneUpdateTime = 1500;
unsigned long lastTempRequest = 0;
int conversionDelay = 900;
bool  waitingForTempsGlobal = false;
bool needReboot = false;
float Temps[15] = {0};
unsigned long lastUpdated[15] = {0};
const word minUpdateTime = 60000;

void OneWireCheck(){
    if (waitingForTempsGlobal && millis() - lastTempRequest >=  conversionDelay ) {
      for(int i=0;i<sensors.getDeviceCount(); i++)  {
          sensors.getAddress(tempDeviceAddress, i);
          float Temp = sensors.getTempC(tempDeviceAddress);
          if ((Temps[i] != Temp || millis() - lastUpdated[i] >= minUpdateTime) && sensors.validAddress(tempDeviceAddress)) {
            Temps[i] = Temp;
            Send(BoardAdd);
            Send(" Rom ");
            for (uint8_t i = 0; i < 8; i++)
              {
              if (tempDeviceAddress[i] < 16) Send("0");
              Send(tempDeviceAddress[i]);
              }
            Send(" ");
            Send(Temp);
            Send();
            lastUpdated[i]=millis();
          }
      }
    waitingForTempsGlobal =false;
    }
  if (!waitingForTempsGlobal && millis() - lastTempRequest > OneUpdateTime){
    lastTempRequest = millis();
    waitingForTempsGlobal =true;
    sensors.requestTemperatures(); 
  }
}
//******************************************************************************



//**********************************Send Data***********************************

bool UdpSend = false;

void SendConnect()
{
#if ISIP == 0
  Serial.print("Connect ");
  Serial.println(BoardAdd);
#else
  if (UdpSend == false) {
    UdpSend=true;
    Udp.beginPacket(ServerIP,ServerPort);  //First send a connect packet to the dynamic IP stored in eeprom
    Udp.print("Connect ");
    Udp.print(BoardAdd);
    Udp.endPacket();
    if (ServerIP!=HomeseerIP) {
      Udp.beginPacket(HomeseerIP,ServerPort);  //Then if the stored value doesn't match the pre-specified one, send a connect packet there also
      Udp.print("Connect ");
      Udp.print(BoardAdd);
      Udp.endPacket();
    }
    UdpSend=false;
  }
#endif
}

void Send(byte Data)
{
#if ISIP == 0
  Serial.print(Data);
#else 
  if (UdpSend == false){
    UdpSend = true;
    Udp.beginPacket(Udp.remoteIP(), ServerPort);
    Udp.print(Data);
  }
  else{
    Udp.print(Data);
  }
#endif
}
void Send(long Data)
{
#if ISIP == 0
  Serial.print(Data);
#else 
  if (UdpSend == false){
    UdpSend = true;
    Udp.beginPacket(Udp.remoteIP(), ServerPort);
    Udp.print(Data);
  }
  else{
    Udp.print(Data);
  }
#endif
}

void Send(int Data)
{
#if ISIP == 0
  Serial.print(Data);
#else 
  if (UdpSend == false){
    UdpSend = true;
    Udp.beginPacket(Udp.remoteIP(), ServerPort);
    Udp.print(Data);
  }
  else{
    Udp.print(Data);
  }
#endif
}

void Send(char* Data)
{
#if ISIP == 0
  Serial.print(Data);
#else 
  if (UdpSend == false){
    UdpSend = true;
    Udp.beginPacket(Udp.remoteIP(), ServerPort);
    Udp.print(Data);
  }
  else{
    Udp.print(Data);
  }
#endif
}


void Send(float Data)
{
#if ISIP == 0
  Serial.print(Data);
#else 
  if (UdpSend == false){
    UdpSend = true;
    Udp.beginPacket(Udp.remoteIP(), ServerPort);
    Udp.print(Data);
  }
  else{
    Udp.print(Data);
  }
#endif
}

void Send()
{
#if ISIP == 0
  Serial.println();
#else 
  Udp.endPacket();
  UdpSend = false;
#endif

}



//*****************************Data Input********************************
/*

Used Data Input Cases
D Disconnect
r reset
F PWM Fade Time Set
P PWM State Set
K Keepalive
W OneWire Pin Set
p PinMode set PWM
a PinMode AnalogInverted Set
A PinMode Analog Input Set
I PinMode Digital Input Set
O PinMode Output Set
d Input debounce time set
S Servo set Pos
s PinMode Servo set
C Connect request
c Connection established - report current status
X Board PinMode Reset
*/

void DataEvent() {

  if (Byte1 == BoardAdd) {

    switch (Byte2) {

    case 'X':
      NoOfInPins=0;
      NoOfServos=0;
      NoOfAnalogPins =0;
      NoOfPwmPins=0;
      break;

    case 'c':
      IsConnected = true;
#if ISIP == 1
      if (Udp.remoteIP() != ServerIP) {
        ServerIP=Udp.remoteIP();
        EEPROM.write(2,ServerIP[0]);
        EEPROM.write(3,ServerIP[1]);
        EEPROM.write(4,ServerIP[2]);
        EEPROM.write(5,ServerIP[3]);
      }     
#endif
      for (count=0;count<NoOfInPins;count++) { 
        int pinread;
        pinread=digitalRead(InPinArray[count]);
        Send(BoardAdd);
        Send(" I ");
        Send(count+1);
        Send(" ");
        Send(pinread);
        Send();
        InStateArray[count] = pinread;
        delay(100);
      }
      break;

    case 'C':
      Send("Version ");
      Send(BoardAdd);
      Send(" ");
      Send(Version);
      Send(" HS3");
      Send();
      delay(100);
      Send("Connected ");
      Send(BoardAdd);
      Send();
      delay(100);
      IsConnected = false;
      break;


    case 's':   
      ServoPinArray[Byte3-1] = Byte4;
      myservo[Byte3-1].attach(Byte4);
      //ServoPosArray[count] = 90;
      ServoSpeedArray[Byte3-1] = 0;
      if (Byte3 > NoOfServos){
        NoOfServos = Byte3;
      }
      break; 

    case 'S':   
      ServoPosArray[Byte3-1] = Byte4;
      break; 

    case 'd':
      Debounce = Byte3;
      break; 

    case 'O':
      pinMode(Byte3, OUTPUT);
      digitalWrite(Byte3, Byte4);
      break; 

    case 'I':
      pinMode(Byte4, INPUT);
      digitalWrite(Byte4, HIGH); 
      if (Byte3 > NoOfInPins){
        NoOfInPins = Byte3;
      }
      InPinArray[Byte3-1] = Byte4;
      break; 

    case 'A':
      pinMode(Byte4, INPUT);
      if (Byte3 > NoOfAnalogPins){
        NoOfAnalogPins = Byte3;
      }
      AnalogueDelay[Byte3-1] = Byte5;
      AnalogPinArray[Byte3-1] = Byte4;
      PrevAnalogeMillis[Byte3-1]= 0;
      break; 

    case 'a':
    bitWrite(AnalogueInvert,Byte3-1,Byte4);
      break; 

    case 'p':
      pinMode(Byte4, OUTPUT);
      if (Byte3 > NoOfPwmPins){
        NoOfPwmPins = Byte3;
      }
      PwmPinArray[Byte3-1] = Byte4;
      break;   

    case 'W':
      if (OneWirePin != Byte3) {
          OneWirePin = Byte3;
          EEPROM.write(1,OneWirePin);
          needReboot = true;
          }
      if (TEMPERATURE_PRECISION != Byte4) {
        TEMPERATURE_PRECISION = Byte4;
        EEPROM.write(6,TEMPERATURE_PRECISION);
        needReboot  = true;
         }
      if (needReboot) { resetFunc(); }
      sensors.begin();
      sensors.setResolution(TEMPERATURE_PRECISION);
      sensors.setWaitForConversion(false);
      if (sensors.isParasitePowerMode()) { sensors.setCheckForConversion(false); }

      break; 

    case 'K':
      delay(200);
      Send("Alive ");
      Send(BoardAdd);
      Send();
#if ISIP == 1
      if (Udp.remoteIP() != ServerIP) {
        ServerIP=Udp.remoteIP();
        EEPROM.write(2,ServerIP[0]);
        EEPROM.write(3,ServerIP[1]);
        EEPROM.write(4,ServerIP[2]);
        EEPROM.write(5,ServerIP[3]);
      }     
#endif
      break; 

    case 'P':
      PwmStateArray[Byte3-1] = Byte4;
      break;

    case 'F':
      PwmFadeTime[Byte3-1] = Byte4;
      break; 
      
    case 'r':
      Send("Reseting ");
      Send();
      delay(200);
      resetFunc();  //call reset
      break; 

    case 'D':
      IsConnected = false;
      break;   

    }
  }
}

//*****************************Serial Event*********************************
void serialEvent() {
  while (Serial.available() > 0) {
    delay(17);
    Byte1 = Serial.parseInt();
    Serial.read();  
    Byte2 = Serial.read(); 
    Byte3 = Serial.parseInt();
    Byte4 = Serial.parseInt();
    Byte5 = Serial.parseInt();
    DataEvent();
  }
}



//*****************************Setup Void*********************************
void setup() {
  for (count=0;count<NoOfPwmPins;count++) { 
    PwmFadeTime[count] = 0;
    PwmStateArray[count] = 0;
    fadeValue[count] = 0;
  }
//*****************************EEPROM Setup******************************

  //***************************End EEPROM Setup
     switch (TEMPERATURE_PRECISION)
    {
    case 9:
        conversionDelay = 94;
    case 10:
        conversionDelay = 188;
    case 11:
        conversionDelay = 375;
    case 12:
        conversionDelay = 750;
    }


#if ISIP == 1
  if (EEpromVersion!=22 && EEpromVersion != 37) {
    ServerIP=HomeseerIP;
    EEPROM.write(2,ServerIP[0]);
    EEPROM.write(3,ServerIP[1]);
    EEPROM.write(4,ServerIP[2]);
    EEPROM.write(5,ServerIP[3]);
     EEpromVersion=22;
 }
  if (EEpromVersion == 22) {
    EEPROM.write(6,9);
    EEpromVersion=37;
  }
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);
  Udp.setTimeout(0);
#else
  if (EEpromVersion != 37) {
    EEPROM.write(6,9);
    EEpromVersion=37;
  }
  Serial.begin(115200);
  Serial.flush();
  Serial.setTimeout(0);
#endif
  EEPROM.write(250,EEpromVersion); //Store the version where the eeprom data layout was last changed
  delay(1000);
  IsConnected = false;
  SendConnect();
}



//*****************************Loop Void*********************************
void loop() {
#if ISIP == 1
  UDPCheck();
#endif
  if (IsConnected == true)
  {
    InputCheck();
    AnalogueCheck();
    PWMCheck();
    ServoCheck();
    if (OneWirePin > 0){  
      OneWireCheck();
    }
  }
}

//****************************************End**************************************************
