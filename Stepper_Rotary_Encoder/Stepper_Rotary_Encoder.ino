#include <Encoder.h>

// This Arduino example demonstrates bidirectional operation of a 
// 28BYJ-48, using a ULN2003 interface board to drive the stepper
// with a red/green rotary encoder with push button to provide
// feedback on direction and speed via the red/green LED, alter
// the direction & speed of the stepper, or click the encoder to 
// reset the encoder to 0 essentially stopping the stepper.
// The 28BYJ-48 motor is a 4-phase, 8-beat motor, geared down by
// a factor of 68. One bipolar winding is on motor pins 1 & 3 and
// the other on motor pins 2 & 4. The step angle is 5.625/64 and the 
// operating Frequency is 100pps. Current draw is 92mA. 
//
// Wiring setup
//
// Wire the stepper motor boards + and - to seperate power(up to +12v)
// alternatively you can tie the power into the vin, assuming that the
// connected motor is rated for that voltage. I wouldn't recommend
// tieing it to the +5 as in a stall state you might release the magic
// smoke...and boy is it hard to get that stuff back in there!
// Wire the 1-4 pads on the motor board to A0-A3 on your 'duino.
// Wire in your rotary encoder to D2 and D3, don't forget ground.
// Wire the other ground as well, and tie the Red pin to 5, 
// Green to 6, and the pushbutton to 12.
// All pins are arbitrary EXCEPT the encoder pins. for optimum
// performance, use two interupt pins. If some misses are ok, use
// one interupt pin and one non-interupt.
// If you won't be using the encoder, and don't care about the 
// performance of it, use any two pins you want.
////////////////////////////////////////////////

//declare variables for the motor pins
int motorPin1 = A3;    // Blue   - 28BYJ48 pin 1
int motorPin2 = A2;    // Pink   - 28BYJ48 pin 2
int motorPin3 = A1;    // Yellow - 28BYJ48 pin 3
int motorPin4 = A0;    // Orange - 28BYJ48 pin 4
                        // Red    - 28BYJ48 pin 5 (VCC)
Encoder myEnc(2,3);

int Spd = 0;
int pushbtn = 12;
int encRed = 5;
int encGrn = 6;

int motorSpeed = 3500;  //variable to set stepper speed
byte lookup[9] = {B01000, B01100, B00100, B00110, B00010, B00011, B00001, B01001, B00000};

//////////////////////////////////////////////////////////////////////////////
void setup() {
  //declare the motor pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(encRed, OUTPUT);
  pinMode(encGrn, OUTPUT);
  pinMode(pushbtn, INPUT_PULLUP);
//Serial.begin(115200);
}

//////////////////////////////////////////////////////////////////////////////
void loop(){
  int newSpd = myEnc.read();
  if (newSpd < -100) {myEnc.write(-100);}
 else if (newSpd > 100) {myEnc.write(100);}
/*
Serial.print("New Speed-");
Serial.print(newSpd);
Serial.print(" - ");
Serial.println(motorSpeed);
*/
if (newSpd != Spd) {
   Spd=constrain(newSpd,-100,100);  
   if (Spd==0) {
     setOutput(8);
     digitalWrite(encGrn,LOW);
     digitalWrite(encRed,LOW);
    }
   if (Spd <0) {
     motorSpeed = map(Spd,-101,-1,1100,10000);
   analogWrite(encRed,map(Spd,-101,-1,255,1));
   digitalWrite(encGrn,LOW);
   }
   if (Spd >0) {
   motorSpeed = map(Spd,1,101,10000,1100);
    analogWrite(encGrn,map(Spd,1,101,1,255));
   digitalWrite(encRed,LOW);
    }
//Serial.print(" - ");
//Serial.println(motorSpeed);
 }
   if (Spd < 0) {clockwise();}
   else if (Spd > 0) {anticlockwise();}
//anticlockwise();
if (digitalRead(pushbtn) == LOW) {
  myEnc.write(0);
  Spd=0;
     setOutput(8);
     digitalWrite(encGrn,LOW);
     digitalWrite(encRed,LOW);
}
}

//////////////////////////////////////////////////////////////////////////////
//set pins to ULN2003 high in sequence from 1 to 4
//delay "motorSpeed" between each pin setting (to determine speed)
void anticlockwise()
{
  for(int i = 0; i < 8; i++)
  {
   // Serial.println("anticlockwise");
    setOutput(i);
    delayMicroseconds(motorSpeed);
  }
}

void clockwise()
{
  for(int i = 7; i >= 0; i--)
  {
   // Serial.println("clockwise");
    setOutput(i);
    delayMicroseconds(motorSpeed);
  }
}

void setOutput(int out)
{
  digitalWrite(motorPin1, bitRead(lookup[out], 0));
  digitalWrite(motorPin2, bitRead(lookup[out], 1));
  digitalWrite(motorPin3, bitRead(lookup[out], 2));
  digitalWrite(motorPin4, bitRead(lookup[out], 3));
}
