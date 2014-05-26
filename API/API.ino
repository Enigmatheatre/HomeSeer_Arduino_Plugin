//For serial set to 0 and for Ethernet set to 1
#define ISIP 0

//Do NOT modify these
#if ISIP == 1
#include <SPI.h>       
#include <Ethernet.h>
#include <EthernetUdp.h> 
#endif
/************************************************************
 *Arduino to Homeseer 3 Plugin API writen by Enigma Theatre.*
 * V1.0.0.23                                                *
 *                                                          *
 *******Change the values below only*************************
 */


//Address of the board.
const byte BoardAdd = 1;

#if ISIP == 1
// Enter a MAC address and IP address for your board below.
const byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// The IP address will be dependent on your local network.
IPAddress ip(192,168,0,100);     //IP entered in HS config.

const unsigned int localPort = 9000;      //port entered in HS config.

const IPAddress HomeseerIP(192,168,0,123); //Homeseer IP address
#endif


//************Do not change anything in Here*****************
int FromHS[10];                                          // *
boolean IsConnected = false;                             // *
#if ISIP == 1                                            // *
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];               // *
EthernetUDP Udp;                                         // *
const unsigned int ServerPort = 8888;                    // *
#endif                                                   // *
//***********************************************************


void setup() {
  HSSetup();
  //************************
  //Add YOUR SETUP HERE;
  //************************




}




void loop() {
#if ISIP == 1
  IsUDP();
#endif

    //************************
    //Add YOUR CODE HERE;
    //************************
    /* To Send Data to Homeseer use SendToHS(Device,Value)
     Eg.. SendToHS(1,200); where 1 is the API device in homeseer and 200 is the value to send
     To Recieve data from Homeseer look up the FromHS array that is updated when the device value changes.
     Eg.. FromHS[5] would be the data from API Output device 5
     All code that is located just below this block will execute regardless of connection status!
     You can include SendToHS() calls, however when there isn't an active connection, it will just return and continue.
     If you only want code to execute when HomeSeer is connected, put it inside the if statement below.
     */

/*Execute regardless of connection status*/


 if (IsConnected == true) {
   /*Execute ONLY when HomeSeer is connected*/

  }
}


























































const char* Version = "API1.0.0.23";

byte Byte1,Byte2,Byte3;
int Byte4,Byte5;


void HSSetup() {

#if ISIP == 1
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);
  Udp.setTimeout(0);
  delay(1000);
  Udp.beginPacket(HomeseerIP, ServerPort);
  Udp.print("Connect ");
  Udp.println(BoardAdd);
  Udp.endPacket();
#else
  Serial.begin(115200);
  Serial.flush();
  Serial.setTimeout(0);
  delay(1000);
  Serial.print("Connect ");
  Serial.println(BoardAdd);
#endif

  IsConnected = false;

}

#if ISIP == 1
void IsUDP(){
  int packetSize = Udp.parsePacket();
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

#else
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
#endif
void DataEvent() {



  if (Byte1 == BoardAdd) {
    switch (Byte2) {

    case 'c':
      IsConnected = true;
      break;

    case 'C':   
#if ISIP == 1
      Udp.beginPacket(Udp.remoteIP(), ServerPort);
      Udp.print("Version ");
      Udp.print(BoardAdd);
      Udp.print(" ");
      Udp.print(Version);
      Udp.println(" HS3");
      Udp.endPacket();

      Udp.beginPacket(Udp.remoteIP(), ServerPort);
      delay(100);
      Udp.print("Connected ");
      Udp.println(BoardAdd);
      Udp.endPacket();
#else
      Serial.print("Version ");
      Serial.print(BoardAdd);
      Serial.print(" ");
      Serial.print(Version);
      Serial.println(" HS3"); 
      delay(100);
      Serial.print("Connected ");
      Serial.println(BoardAdd);
#endif
      delay(100);
      IsConnected = false;
      break;

    case 'K':
      delay(200);
#if ISIP == 1
      Udp.beginPacket(Udp.remoteIP(), ServerPort);
      Udp.print("Alive ");
      Udp.println(BoardAdd);
      Udp.endPacket();
#else     
      Serial.print("Alive ");
      Serial.println(BoardAdd);
#endif
      break; 

    case 'O':
      FromHS[Byte3] = Byte4;
      break; 

    case 'D':
      IsConnected = false;
      break;   
    }
  }
}

void SendToHS(byte Device, long Data){
if (IsConnected == true) {
#if ISIP == 1
  Udp.beginPacket(Udp.remoteIP(), ServerPort);
  Udp.print(BoardAdd);
  Udp.print(" API ");
  Udp.print(Device);
  Udp.print(" ");
  Udp.print(Data);
  Udp.endPacket();
#else
  Serial.print(BoardAdd);
  Serial.print(" API ");
  Serial.print(Device);
  Serial.print(" ");
  Serial.println(Data);
#endif
}
}

