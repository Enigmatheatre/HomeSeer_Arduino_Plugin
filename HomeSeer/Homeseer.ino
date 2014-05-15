#include <EEPROM.h>


/********************************************************
 *Arduino to Homeseer 3 Plugin writen by Enigma Theatre.*
 * V1.0.0.22                                            *
 *                                                      *
 *******Do not Change any values below*******************
 */


//Global Variables
byte BoardAdd = 1;
#define ISIP 0
byte Byte1,Byte2,Byte3;
int Byte4,Byte5;
char* Version = "1.0.0.22";
bool IsConnected = false;
void(* resetFunc) (void) = 0; 


//******************************Ethernet Setup*****************************
#if ISIP == 1

#include <SPI.h>       
#include <Ethernet.h>
#include <EthernetUdp.h> 

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192,168,0,145);     //IP entered in HS config.
unsigned int localPort = 9000;      //port entered in HS config.
IPAddress HomeseerIP(192,168,0,20); //Homeseer IP address

char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,

EthernetUDP Udp;
unsigned int ServerPort = 8888;     // port to Send To

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
byte InPinArray[] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte Debounce = 30;
byte count = 0; 
byte NoOfInPins = 0; 
int InStateArray  [(sizeof(InPinArray) / 2)];
unsigned long PrevDebounce [(sizeof(InPinArray) / 2)];

void InputCheck(){
  byte pinread;
  for (count=0;count<NoOfInPins;count++) {
    if(millis() - PrevDebounce[count] > Debounce){
      pinread = (digitalRead (InPinArray[count]));
      if (InStateArray[count] != pinread){

        InStateArray[count] = pinread;
        PrevDebounce[count] = millis();
        SendByte(BoardAdd); 
        SendChar(" I ");
        SendByte(count+1); 
        SendChar(" ");  
        SendByte(pinread); 
        Sendln();
      }
    }
  }
}

//*******************************Analogue Setup****************************
byte  AnalogPinArray[] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int AnalogueDelay[] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  
word AnalogueInvert = 0;

byte NoOfAnalogPins =0;
int AnalogStateArray[(sizeof(AnalogPinArray) / 2)];
unsigned long PrevAnalogeMillis[sizeof(AnalogPinArray) / 2]; 

  void  AnalogueCheck(){
    for (count=0;count<NoOfAnalogPins;count++) {
      if(millis() - PrevAnalogeMillis[count] > AnalogueDelay[count]) {
        PrevAnalogeMillis[count] = millis(); 
        if (AnalogStateArray[count] != (analogRead (AnalogPinArray[count]))){
          AnalogStateArray[count] = (analogRead (AnalogPinArray[count]));
          SendByte(BoardAdd); 
          SendChar(" A ");
          SendByte(count+1); 
          SendChar(" ");  
          if bitRead(AnalogueInvert,count = 1){
            // InvertedValue = map(AnalogStateArray[count], 0, 1023, 1023, 0);
            SendByte(map(AnalogStateArray[count], 0, 1023, 1023, 0)); 
            Sendln();
          }
          else{
            SendByte(AnalogStateArray[count]);
            Sendln();
          }
        }
      }
    }
  }


//*****************************PWM Setup****************************
byte PwmPinArray[] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte NoOfPwmPins = 0;
int PwmStateArray[(sizeof(PwmPinArray) / 2)];
int PwmFadeTime[(sizeof(PwmPinArray) / 2)];
byte fadeTarget[(sizeof(PwmPinArray) / 2)];
byte fadeValueTweened[(sizeof(PwmPinArray) / 2)];
byte fadeValue[(sizeof(PwmPinArray) / 2)];
unsigned long fadeTimerLast[(sizeof(PwmPinArray) / 2)];

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
//bool UseServo = false;
byte ServoPinArray[8] = {
  0};
byte NoOfServos = 0;
Servo myservo[(sizeof(ServoPinArray) / 2)];
int ServoPosArray[(sizeof(ServoPinArray) / 2)];
int ServoOldPosArray[(sizeof(ServoPinArray) / 2)];
int ServoSpeedArray[(sizeof(ServoPinArray) / 2)];

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
int OneWirePin = EEPROM.read(1);
#define TEMPERATURE_PRECISION 7
OneWire oneWire(OneWirePin);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
unsigned long PrevOneMillis = 0;
int OneUpdateTime = 2500;
float onewiretemps[]={
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void OneWireCheck(){
  if (millis() - PrevOneMillis > OneUpdateTime){
    PrevOneMillis = millis();
    sensors.requestTemperatures(); 

    for(int i=0;i<sensors.getDeviceCount(); i++)
    {

      if(sensors.getAddress(tempDeviceAddress, i)){
        float Temp = sensors.getTempC(tempDeviceAddress);
        if (onewiretemps[i] != Temp){
          onewiretemps[i] = Temp;              
          SendByte(BoardAdd);
          SendChar(" Rom ");
          for (uint8_t i = 0; i < 8; i++)
          {
            if (tempDeviceAddress[i] < 16) SendChar("0");
            SendByte(tempDeviceAddress[i]);
          }
          SendChar(" ");
          SendFloat(Temp);
          Sendln();
        }

      }
    }
  }
}
//******************************************************************************



//**********************************Send Data***********************************

bool UdpSend = false;

void SendByte(int Data)
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

void SendChar(char* Data)
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


void SendFloat(float Data)
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

void Sendln()
{
#if ISIP == 0
  Serial.println();
#else 
  Udp.endPacket();
  UdpSend = false;
#endif

}



//*****************************Data Input********************************
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

      for (count=0;count<NoOfInPins;count++) { 
        SendByte(BoardAdd);
        SendChar(" I ");
        SendByte(count+1);
        SendChar(" ");
        SendByte(digitalRead (InPinArray[count]));
        Sendln();
        InStateArray[count] = (digitalRead (InPinArray[count]));
        delay(100);
      }
      break;

    case 'C':
      SendChar("Version ");
      SendByte(BoardAdd);
      SendChar(" ");
      SendChar(Version);
      SendChar(" HS3");
      Sendln();
      delay(100);
      SendChar("Connected ");
      SendByte(BoardAdd);
      Sendln();
      delay(100);
      IsConnected = false;
      break;


    case 's':   
      ServoPinArray[Byte3-1] = Byte4;
      myservo[Byte3-1].attach(Byte4);
      //      ServoPosArray[count] = 90;
      ServoSpeedArray[Byte3-1] = 0;

      if (Byte3 > NoOfServos){
        NoOfServos = Byte3;
        // UseServo = true;
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
      //AnalogueInvert[Byte3-1] = Byte4;
      break; 

    case 'p':
      pinMode(Byte4, OUTPUT);
      if (Byte3 > NoOfPwmPins){
        NoOfPwmPins = Byte3;
      }
      PwmPinArray[Byte3-1] = Byte4;
      break;   

    case 'W':
      OneWirePin = Byte3;
      EEPROM.write(1,Byte3);

      sensors.begin();
      //numberOfDevices = sensors.getDeviceCount();
      for(int i=0;i<sensors.getDeviceCount(); i++)
      {
        if(sensors.getAddress(tempDeviceAddress, i))
        {
          sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
        }
      }
      break; 

    case 'K':
      delay(200);
      SendChar("Alive ");
      SendByte(BoardAdd);
      Sendln();
      break; 

    case 'P':
      PwmStateArray[Byte3-1] = Byte4;
      break;

    case 'F':
      PwmFadeTime[Byte3-1] = Byte4;
      break; 

    case 'R':
      ServoPosArray[Byte3-1] = Byte4;
      break; 

    case 'r':
      SendChar("Reseting ");
      Sendln();
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

#if ISIP == 1
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);
  Udp.setTimeout(0);
#else
  Serial.begin(115200);
  Serial.flush();
  Serial.setTimeout(0);
#endif
  delay(1000);
  SendChar("Connect ");
  SendByte(BoardAdd);
  Sendln();
  IsConnected = false;
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
