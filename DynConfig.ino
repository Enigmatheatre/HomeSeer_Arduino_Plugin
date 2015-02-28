/*
  This code is a part of the Cotton project.

  Copyright (c) 2011 
    * Ivan Antoniyuk, ivan.antonyuck@gmail.com
    * Mansur Ziyatdinov, gltronred@gmail.com
    * Eugene Katsevman, eugene.katsevman@gmail.com

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

/* And now for something completely different.

Main point of all those code line below is to configure list of external 
devices "on the fly", without firmware reburning, and to have a way to control those
devices.
Firmware have some common ground with SimpleMessageSystem, though it was completely coded from scratch.
This approach allows us to eliminate the need for additional comm software at the PC side.

Here is an example of PC-Arduino dialog
def servo hor 10       ; we have a servo motor, connected to pin 10. We want it to be named "hor"
def servo vert 11      : and we have a servo on pin 11. We want to call it "vert"
def led led 13         ; we want to control onboard led (pin 13)
save                   ; store definitions to eeprom, so we'll have same config after reset
                       ; now we can issue commands
hor 170                ; rotate first servo to the left
vert 90                ; set second servo to its central position
led on                 ; turn the led on
led off                ; turn the led off 
*/

#include <Servo.h>
#include <EEPROM.h>

#include <stdlib.h> // for malloc and free

// local new and delete. We have to define it somewhere.
void * operator new(size_t size) { return malloc(size); }
void   operator delete(void* ptr) { free(ptr); } 


/***
 * Protocol parameters:
 * - MAX_CMD_LEN - maximum length of command
 *     Please, make sure that any command will not exceed this limit!
 * - MAX_QUIET_TIME - how much periods will reader wait before exiting
 * - READER_PERIOD_TIME - length of reader time periods (see prev. parameter)
 * - MAGICK - a little magick. Used as eeprom first byte, if configuration is present. 
 * - SERIAL_SPEED - serial baudrate from 9600 to 115200
 * - SENSOR_PAUSE - sensor report period in milliseconds
 */
const int MAX_CMD_LEN = 20;
const int MAX_QUIET_TIME = 2;
const int READER_PERIOD_TIME = 2;
const int MAGICK=42;
const long SERIAL_SPEED = 115200;
const int SENSOR_PAUSE = 500; 

// common namespace
namespace cotton {

// we skip whitespaces, starting at specified position
// and return index of first non-whitespace character
int skipws(const char * str, int start){
  int i = start;
  while (str[i] && str[i] == ' ') i++;
  return i;
}

// now we skip all non-whitespace from some position
// and return index of first whitespace character
int getSpaceIndex(const char * str, int start){
  int i = start;
  while (str[i] && str[i] != ' ') i++;
  return i;
}

// here we parse string, skipping all non whitespace chars
// and then copy non-ws char sequence to param.
// we return start or first non-ws index
int getParam(char * param, const char * str, int start){

  int bound = getSpaceIndex(str, start);

  memset(param, 0, MAX_CMD_LEN);
  strncpy(param,str + start, bound - start);

  return skipws(str, bound);
}

// Command class defines an interface to named command.
// Command is parametrized by it's name. Its children are parametrized  by
// some additional params, e.g. pin numbers or time periods.
// Commands are used to bind some name to implementations. For example, we can bind "led" to CommandLed and then use "led on" and "led off" to manage
// some LED.
// Commands can serialize themselves via dump() (see CommandStore)

class Command {
public:
  char * name;
  Command(char * _name){
    name = (char *) malloc(strlen(name) + 1);
    strcpy(name, _name);
  };
  ~Command(){
    free(name);
  }
  virtual void action(const char * arg){};
  virtual void ontimer(){};
  virtual void dump(char * str){};
};

// CommandLed is used for managing LEDs with on and off subcommands
// Parametrized with pin number (digital or PWM)
class CommandLed : public Command {
public:
  int pin;
  CommandLed(char * _name, int _pin): Command(_name){
    pin = _pin;
    pinMode(pin, OUTPUT);
  };
  void action (const char * arg){
      char param[MAX_CMD_LEN];
      getParam(param, arg, 0);
      if (strcmp(param, "on") == 0){
         digitalWrite(pin, HIGH);
      }
      else if (strcmp(param, "off") == 0 ) {
          digitalWrite(pin, LOW);
      }; // else do nothing
  };
  void dump(char * str){
    sprintf(str, "led %s %d", name, pin);
  }
};

// Used for servo control
// Parametrized by PWM pin number
// Understands integer angle (0-180) as a parameter 
class CommandServo : public CommandLed {
public:
  Servo servo;
  CommandServo(char * _name, int _pin): CommandLed(_name, _pin){
    servo.attach(pin, 450, 2450);
  };
  void action (const char * arg){
      char param[MAX_CMD_LEN];
      getParam(param, arg, 0);
      int t = atoi(param);
      servo.write(t);
  };
  void dump(char * str){
    sprintf(str, "servo %s %d", name, pin);
  }
};

// Used for "misjustified" servos, that can't be justified.
// Parametrized by PWM pin number and "central" angle.
// If servo is aligned correctly, then center should be equal to 90, otherwise
// it should specify correct "central" position.
// Understands same parameter, as its predecessor CommandServo.
class CommandServoCorrected : public CommandServo {
public:
  int center;
  CommandServoCorrected(char * _name, int _pin, int _center): CommandServo(_name, _pin){
    center = _center;
  };
  void action (const char * arg){
      char param[MAX_CMD_LEN];
      getParam(param, arg, 0);
      int t = center - 90 + atoi(param);
      servo.write(t);
  };
  void dump(char * str){
    sprintf(str, "servo_c %s %d %d", name, pin, center);
  }
};


// CommandMotor is user for DC motor control. It is parametrized by
// PWM pin number and direction pin number.
// Motors understand one integer parameter (from -254 to 255), where 
// 0 is full stop , -254 and 255 correspond to full back and full forward 
// respectively
class CommandMotor : public Command {
public:
  int pwm_pin;
  int rev_pin;
  CommandMotor(char * _name, int _pwm_pin, int _rev_pin): Command(_name){
    pwm_pin = _pwm_pin;
    pinMode(pwm_pin, OUTPUT); 
    rev_pin = _rev_pin;
    pinMode(rev_pin, OUTPUT); 
  };
  void action (const char * arg){
      char param[MAX_CMD_LEN];
      getParam(param, arg, 0);
      int t = atoi(param); 
      Serial.println(rev_pin);
      Serial.println(pwm_pin);
      Serial.println(t);
      if (t > 0){
        digitalWrite(rev_pin, LOW);
        analogWrite(pwm_pin, t);
      } else  {
        digitalWrite(rev_pin, HIGH);
        analogWrite(pwm_pin, 255 + t);
      }  
  };
  void dump(char * str){
    sprintf(str, "motor %s %d %d", name, pwm_pin, rev_pin);
  }  
};

// CommandSensor is used to get values from
// ADC pins A0-A7.
// It is parametrized by ADC pin number only
// It understands one optional parameter - start or stop
// Example:
// >def sensor temp 0
// >temp
// <130
// >temp start
// <130
// <120
// <140
// <130
// >temp stop             
class CommandSensor : public Command {
public:
  int lasttime;
  int pin;
  bool automatic;
  CommandSensor(char * _name, int _pin): Command(_name){
    pin = _pin;
    automatic = false;
    lasttime = millis();
  };
  void action (const char * arg){
    char param[MAX_CMD_LEN];
    getParam(param, arg, 0);
    if (strcmp(param, "start") == 0)
      automatic = true;
      else
        if(strcmp(param, "stop") == 0)
          automatic = false;
          else
            write();

  };
  void write(){
    Serial.print(name);
    Serial.print(" ");
    Serial.println(analogRead(pin));
  };
  void ontimer(){
    if (automatic){
      int time;
      time = millis();
      if (lasttime < time - SENSOR_PAUSE){
        lasttime = time;
        write();
      }
    }
  }
  void dump(char * str){
    sprintf(str, "sensor %s %d", name, pin);
  }  
};

const int MAX_CMD_COUNT = 16;
int CMD_COUNT = 0;
const int BUILTIN_COUNT = 4; 
Command * builtins[BUILTIN_COUNT];
Command * commands[MAX_CMD_COUNT];

// Do you really need a comment for that?
int AddCommand(Command * cmd){
  // check, whether we can add command 		
  if (CMD_COUNT < MAX_CMD_COUNT - 1){
    commands[CMD_COUNT++] = cmd;
    return CMD_COUNT - 1;
    }
  else{
    Serial.print("No room for new command.");
    Serial.print(cmd->name);
    return -1;
  }
};

// Command for command dynamic definition
// parses a definition and constructs corresponding Command object.
// Then tries to add it to commands[]
class CommandDef : public Command {
public:
  CommandDef(char * _name): Command(_name){};

  void action (const char * arg){
    // let's parse!
    Command * cmd = 0;
    char name [MAX_CMD_LEN];
    int end1 = getParam(name,arg, 0);
    if (strcmp(name, "led") == 0){
      end1 = getParam(name, arg, end1);
      char pin [MAX_CMD_LEN];
      end1 = getParam(pin, arg, end1);
      cmd = new CommandLed(name, atoi(pin));
    } else
    if (strcmp(name, "servo") == 0){
      end1 = getParam(name, arg, end1);
      char pin [MAX_CMD_LEN];
      end1 = getParam(pin, arg, end1);
      cmd = new CommandServo(name, atoi(pin));
    } else
    if (strcmp(name, "servo_c") == 0){
      end1 = getParam(name, arg, end1);
      char pin [MAX_CMD_LEN];
      end1 = getParam(pin, arg, end1);

      char center[MAX_CMD_LEN];
      end1 = getParam(center, arg, end1);
      cmd = new CommandServoCorrected(name, atoi(pin), atoi(center));
    } else
    if (strcmp(name, "motor") == 0){
      end1 = getParam(name, arg, end1);
      char pin1 [MAX_CMD_LEN];
      end1 = getParam(pin1, arg, end1);
      char pin2 [MAX_CMD_LEN];
      end1 = getParam(pin2, arg, end1);
      cmd = new CommandMotor(name, atoi(pin1), atoi(pin2));
    } else
    if (strcmp(name, "sensor") == 0){
      end1 = getParam(name, arg, end1);
      char pin [MAX_CMD_LEN];
      end1 = getParam(pin, arg, end1);
      cmd = new CommandSensor(name, atoi(pin));
    };

    if (cmd){
      if (-1 == AddCommand(cmd))
        delete cmd;
    }
  };
};


class CommandClear : public Command {
public:
  CommandClear(char * _name): Command(_name){
  };
  void action (const char * arg){
    for (int i = 0; i < CMD_COUNT; i++)
      delete commands[i];
    CMD_COUNT=0;
  };
};


// Store all dynamic definitions to eeprom.
// First write magick to first eeprom byte, then
// write all commmand dumps.
class CommandSave : public Command {
public:
  CommandSave(char * _name): Command(_name){
  };
  void action (const char * arg){
    Serial.println("dumping commands");
    int address = 0;
    EEPROM.write(address++, MAGICK);
    for (int i = 0; i < CMD_COUNT; i++){
      char dump[MAX_CMD_LEN + 1];
      memset(dump, 0, MAX_CMD_LEN); 
      commands[i]->dump(dump);
      int len = strlen(dump);
      Serial.println(address);
      Serial.println(dump);
      for (int j = 0; j < len; j++)
        EEPROM.write(address++, dump[j]);
      EEPROM.write(address++, 0);  
    }
    EEPROM.write(address, 0);
  };
};

// forward declaration of eval-like function
void execCmd(const char * cmd);

// restore all command definitions from eeprom
class CommandRestore : public Command {
public:
  CommandRestore(char * _name): Command(_name){
  };
  void action (const char * arg){
    int address = 0;
    if (EEPROM.read(address++) == MAGICK)
      while (EEPROM.read(address)){
        char cmd[MAX_CMD_LEN + 1];
        char defcmd[MAX_CMD_LEN + 1];
        char * c = cmd;
        while (*(c++) = EEPROM.read(address++));
        sprintf(defcmd, "def %s", cmd);
        execCmd(defcmd);
    }
  };
};

// TODO: think about speed of working (should we use binary search here?)
bool parseCmd(const char * str, Command * & result, const char * & arg){
  char cmd[MAX_CMD_LEN];
  int end1 = getParam(cmd,str, 0);
  arg = str + end1;
  int index = 0; // not found;
  while(index < BUILTIN_COUNT && strcmp(cmd, builtins[index]->name) != 0){
    index++;
  }

  if (index==BUILTIN_COUNT)
    result = 0;
  else
    {
      result = builtins[index];
      return true;
    };
  if (result == 0){
    index = 0;  
    while(index < CMD_COUNT && strcmp(cmd,commands[index]->name) != 0){
      index++;
    }
    if (index==CMD_COUNT)
      result = 0;
    else{
      result = commands[index];
      return true;
    }
  }
  return false;
};

// we have The Eval here.
void execCmd(const char * cmd){
    const char * arg;
    cotton::Command * command;   
    if (!cotton::parseCmd(cmd, command, arg)) {
      Serial.print("UNKNOWN CMD: ");
      Serial.println(cmd);
    }else{
      Serial.print("Got command: ");
      Serial.print(command->name);
      Serial.print("(");
      Serial.print(arg);
      Serial.println(")");
      command->action(arg);
    }
}
/***
 * Reads a command into buffer[]
 * Command is one line, followed by \x0d\x0a or \x0a only 
 */
void readln( char buffer[] ) {
  char c;
  int i = 0;
  int quiet = 0; // quietness periods
  int exit = 0; // exit loop if 1
  while (exit == 0 && quiet++ <= MAX_QUIET_TIME && i <MAX_CMD_LEN){
    if (Serial.available()){
      quiet = 0; // not quiet
      c = Serial.read();
      switch(c){
      case '\r':
	Serial.read();
	exit = 1; 
	break; 
      case '\n':
	exit = 1;
	break;
      default:
	buffer[i++] = c;
	break;
      }
    }
    delay(READER_PERIOD_TIME);
  }
  buffer[i] = NULL;
};

};

/***
 * Main program begins here
 */

void setup() {
  // we can use it later
  //TCCR1B = TCCR1B & 0b11111000 | 0x01; // 9, 10 
  //TCCR2B = TCCR2B & 0b11111000 | 0x01; // 3, 11

  Serial.begin(SERIAL_SPEED);
  Serial.println("Ready to rock");
  cotton::builtins[0] = new cotton::CommandDef("def"); 
  cotton::builtins[1] = new cotton::CommandClear("clear");
  cotton::builtins[2] = new cotton::CommandSave("save");
  cotton::builtins[3] = new cotton::CommandRestore("restore");
  cotton::execCmd("restore"); // we could do it via builtins
}

/***
 * Main loop
 */
void loop() {
  char cmd[MAX_CMD_LEN + 1];
  if (Serial.available() != 0){
    cotton::readln(cmd);
    cotton::execCmd(cmd);
    }
  for (int i = 0 ; i < cotton::CMD_COUNT; i++){
    cotton::commands[i]->ontimer();
  }  
  }
