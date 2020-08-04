/*
    Intervaluino Plus
    Copyright (C) 2012 Enekochan
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
Intervaluino Plus Version 1.0 by Enekochan

Release Notes:
 Version 1.0 First working version
 Version 0.2 Ported to Arduino 1.0 (with SoftwareSerial in the core provided by Arduiniana http://arduiniana.org/libraries/newsoftserial/ )
 Version 0.1 First not working prototype using NewSoftSerial library

Hardware
--------
Display: Sparkfun 4 character 7 segment LED display serially driven using just the "Rx" input on the module.
See Sparkfun.com product codes COM-09764 to COM-09767 (differ only in color)

Connect one of the above modules to an Arduino with just 3 wires:
- 5v (Vcc) (or 3.3, if you are using a low voltage system. Module is good down to 2.6v)
- Ground (GND)
- Rx pad of the module. Connect to the pin you choose to TRANSMIT data out from the Arduino.

Datasheet: http://www.sparkfun.com/datasheets/Components/LED/7-Segment/SFE-0012-DS-7segmentSerial-v41.pdf

ATTENTION! -> Read the datasheet carefully, there are some commands that control the display that
can change the way it behaves. For example if you send a 0x76 value it will clear the display and
set the cursor to the first digit, even if you sended a single character control byte (0x7B,0x7C,0x7D,0x7E)
before. So, if you are trying to light up the B, C, E, F and G individual segments (to create an H shape)
sending B01110110 at digit 1 executing this:

mySerialPort.write(0x7B);      // Control digit 1
mySerialPort.write(B01110110); // This is 0x76!!!

What you are really doing is sending first a 0x7B and then ¡¡¡a 0x76!!! and that will clear the display
and move cursor to digit 1. ¿Any one knows how to display an H on this display?
For more details see "Special Codes" section in http://www.arunet.co.uk/tkboyd/ec/ec1led4x7ser.htm

*/

// Timer interrupts
#include <avr/interrupt.h>
#include <avr/io.h>

// Serial driver for 4 character 7 segment LED display
#include <SoftwareSerial.h>

#define DEBUG 0 // BEWARE! Activating the debug thru the serial port makes the FastForward not to work as expected.
                // Sending data creates an "artificial" delay and the FastForward function will act slower
#define COMPUTER_SERIAL_PORT_SPEED_BAUD 9600 // Serial port bauds with computer

// Definitions for interrupt timer 2
// (NOTE: Interrupt 2 is used in Arduino PWM library. This may interfere with PWM applications)
#define INIT_TIMER_COUNT 6
#define RESET_TIMER2 TCNT2 = INIT_TIMER_COUNT

// Sparkfun Display pins
#define SERIAL_IN 2  // Digital Pin 2
#define SERIAL_OUT 3 // Digital Pin 3

// Button pins
#define RIGHT_IN 5
#define LEFT_IN 6
#define UP_IN 7
#define DOWN_IN 8

// Optocoupler pins
#define SHUTTER_OUT 9
//#define FOCUS_OUT 10

// Button position in Array of Buttons
#define RIGHT 0
#define LEFT 1
#define UP 2
#define DOWN 3

#define NUM_DIGITS 4  // Display has 4 digits
#define NUM_BUTTONS 4 // 4 Buttons (Right, Left, Up and Down)

// Application current status
#define READING_PICS 0
#define READING_INTERVAL 1
#define TAKING_PICTURES 2

#define SHUTTER_PRESS_DELAY   900 // How long (ms) the shutter will be closed to take a picture
//#define FOCUS_PRESS_DELAY     900 // How long (ms) the focus will be closed
#define FASTFORWARD_DELAY     500 // Time to wait until fastforward is activated
#define FASTFORWARD_REP_DELAY 20  // Delay between each repeated button click

#define ASCII_NUMBER_POSITION 48 // Numbers start at ascii 48, used to convert char representing numbers to actual numbers

// Structure defining a button
struct Button {
  boolean actualState;   // Actual button state
  boolean previousState; // Previous button state
  boolean fastForward;   // Has been the button clicked for more than "FASTFORWARD_DELAY" time?
  long previousMillis;   // Last state verification time
};

// Create the serial channel for the display
SoftwareSerial mySerialPort(SERIAL_IN, SERIAL_OUT);
int dotsLastState = 0;
char actualDisplayValue[NUM_DIGITS] = {'0','0','0','0'};
char pics[NUM_DIGITS] = {'0','0','0','0'};
char interval[NUM_DIGITS] = {'0','0','0','0'};
long debounce = 50;
long actualMillis = 0;
int state = READING_PICS; // Actual state. First we want to know how many pictures to take
int timerMilliseconds = 0;
int timerSeconds = 0;
long shutterMillis = 0;
boolean takingPicture = LOW;
int picsValue = 0; // Value readed for how many pictures to take
int intervalValue = 0; // Value readed for time between every picture
int picturesTakenValue = 0; // How many pics have we taken so far

Button RightButton = {LOW, LOW, LOW, 0};
Button LeftButton = {LOW, LOW, LOW, 0};
Button UpButton = {LOW, LOW, LOW, 0};
Button DownButton = {LOW, LOW, LOW, 0};

Button Buttons[NUM_BUTTONS] = {RightButton, LeftButton, UpButton, DownButton}; 

////////////////////////////////////////////////////////////////////////////////////////////////////
// This function is called 1000 times per second ( http://basbrun.com/2009/07/10/arduino-timer-interrupt/ )
////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(TIMER2_OVF_vect) {
  if (state == TAKING_PICTURES) {
    // Arduino Duemilanove runs at 16 Mhz, so we have 1000 Overflows per second...
    // 1/ ((16000000 / 64) / 256) = 1 / 1000
    RESET_TIMER2;
  
    timerMilliseconds++;
  
    if(timerMilliseconds == 1000) { // Every 1000 miliseconds...
      if (DEBUG) Serial.println("1000 seconds more...");
      timerMilliseconds = 0;
      timerSeconds++;
      if (timerSeconds == intervalValue) { // When we reach the interval value delay
        timerSeconds = 0;
        digitalWrite(SHUTTER_OUT, HIGH); // Take the picture
        takingPicture = HIGH;
        shutterMillis = millis();
        if (DEBUG) {Serial.print("CLICK! Picture ");Serial.print(picturesTakenValue);Serial.print("/");Serial.println(picsValue);};
      }
    }

    // If we are taking pictures and shutter has been pressed for SHUTTER_PRESS_DELAY milliseconds release it
    if (takingPicture == HIGH && millis() - shutterMillis >= SHUTTER_PRESS_DELAY) {
      digitalWrite(SHUTTER_OUT, LOW);
      takingPicture = LOW;
      if(++picturesTakenValue == picsValue) { // If we have already taken all the pics we wanted
        picturesTakenValue = 0;
        timerMilliseconds = 0;
        timerSeconds = 0;
        buttonClick(LEFT); // We fake a LEFT button click to go again into the READING_PICS state
      }
    }
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// setup()
////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Configure serial communication with computer
  Serial.begin(COMPUTER_SERIAL_PORT_SPEED_BAUD);
  
  // Digital INPUT/OUTPUT for Display
  pinMode(SERIAL_IN, INPUT); // Not actually needed... put in to be explicit as to data direction over serial lines
  pinMode(SERIAL_OUT, OUTPUT);
  // Digital INPUTs for buttons
  pinMode(RIGHT_IN, INPUT);
  pinMode(LEFT_IN, INPUT);
  pinMode(DOWN_IN, INPUT);
  pinMode(UP_IN, INPUT);
  // Digital OUTPUTs for optocouplers
  pinMode(SHUTTER_OUT, OUTPUT);
  //pinMode(FOCUS_OUT, OUTPUT);
  
  // Configure 4 digit 7 segmet display to 9600 bps and reset it
  mySerialPort.begin(9600);
  mySerialPort.write("v"); // Reset display module
  
  // First Stage is waiting for the number of pictures to take
  displayText("PICS");
  turnOnDot1();
  
  // Configure display brightness.
  // - If UP is clicked while booting it will be configured high bright
  // - If DOWN is clicked while booting it will be configured low bright
  // - If nothing is clicked it will be configured to medium bright
  if (digitalRead(UP_IN) == HIGH)
    highBright();
  else if (digitalRead(DOWN_IN) == HIGH)
    lowBright();
  else
    mediumBright();
  
  delay(1000); // Wait 1 sec to release buttons
  
  // Configure and activate timer interrupt
  // Timer2 Settings: Timer Prescaler /64,
  TCCR2A |= (1<<CS22);
  TCCR2A &= ~((1<<CS21) | (1<<CS20));
  // Use normal mode
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
  // Use internal clock - external clock not used in Arduino
  ASSR |= (0<<AS2);
  // Timer2 Overflow Interrupt Enable
  TIMSK2 |= (1<<TOIE2) | (0<<OCIE2A);
  RESET_TIMER2;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// loop()
////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  actualMillis = millis();
  readButtonsActualState();
  processButtonEvents();
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Button related functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void readButtonsActualState() {
  Buttons[RIGHT].actualState = digitalRead(RIGHT_IN);
  Buttons[LEFT].actualState = digitalRead(LEFT_IN);
  Buttons[UP].actualState = digitalRead(UP_IN);
  Buttons[DOWN].actualState = digitalRead(DOWN_IN);
}

void processButtonEvents() {
  // Process all button clicks, fastforwards and releases
  for (int i = 0 ; i < NUM_BUTTONS ; i++) {
    // Single button click
    if (Buttons[i].actualState != Buttons[i].previousState && (actualMillis - Buttons[i].previousMillis) >= debounce) {
      Buttons[i].previousState = Buttons[i].actualState;
      Buttons[i].previousMillis = actualMillis;
      if (Buttons[i].actualState == HIGH) {
        buttonClick(i);
      }
    }
    
    // FastForward activation
    if (Buttons[i].actualState == HIGH && Buttons[i].actualState == Buttons[i].previousState && (actualMillis - Buttons[i].previousMillis) >= FASTFORWARD_DELAY) {
      Buttons[i].fastForward = HIGH;
    }
    
    // FastForward
    if (Buttons[i].fastForward && Buttons[i].actualState == HIGH && Buttons[i].actualState == Buttons[i].previousState && (actualMillis - Buttons[i].previousMillis) >= FASTFORWARD_REP_DELAY) {
      Buttons[i].previousMillis = actualMillis;
      buttonClick(i);
    }
  
    // Release button
    if (Buttons[i].actualState == LOW) {
      Buttons[i].fastForward = LOW;
      Buttons[i].previousState = LOW;
      Buttons[i].previousMillis = actualMillis;
      buttonRelease(i);
    }
  }  
}

void buttonClick(int buttonID) {
  switch (buttonID) {
    
    case UP:
      if(state == READING_PICS) {
        add1To(pics, &picsValue);
        displayCounterValue(pics);
      } else if(state == READING_INTERVAL) {
        add1To(interval, &intervalValue);
        displayCounterValue(interval);
      }
      break;
    
    case DOWN:
      if(state == READING_PICS) {
        subtract1To(pics, &picsValue);
        displayCounterValue(pics);
      } else if(state == READING_INTERVAL) {
        subtract1To(interval, &intervalValue);
        displayCounterValue(interval);
      }
      break;
    
    case RIGHT:
      if(state == READING_PICS) {
        // Nothing to do here
      } else if(state == READING_INTERVAL) {
        intervalValue = toInt(interval);
        state = READING_PICS;
        turnOffDot2();
        displayText("PICS");
        delay(1000);
        turnOnDot1();
        // Load interval on display
        displayCounterValue(pics);
      } else if(state == TAKING_PICTURES) {
        state = READING_INTERVAL;
        displayText("PERI");
        turnOffDot1();
        delay(1000);
        turnOnDot2();
        displayCounterValue(interval);
      }
      break;
    
    case LEFT:
      if(state == READING_PICS) {
        picsValue = toInt(pics);
        state = READING_INTERVAL;
        turnOffDot1();
        displayText("PERI");
        delay(1000);
        turnOnDot2();
        displayCounterValue(interval);
      } else if(state == READING_INTERVAL) {
        intervalValue = toInt(interval);
        if (picsValue != 0 && intervalValue != 0) {
          state = TAKING_PICTURES;
          turnOffDot1();
          turnOffDot2();
          displayText("PHOT");
          delay(1000);
          sei(); // Activate interrupts to start taking pictures with the ISR(TIMER2_OVF_vect) function
        }
      } else if(state == TAKING_PICTURES) {
        state = READING_PICS;
        displayText("PICS");
        delay(1000);
        turnOnDot1();
        displayCounterValue(pics);
      }
      break;
  }
  
  if (DEBUG) Serial.println("Button clicked: " + buttonID);
  printDebugData();
}

void buttonRelease(int buttonID) {
  switch (buttonID) {
    case RIGHT:
      break;
    case LEFT:
      break;
    case DOWN:
      break;
    case UP:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// Display functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void displayCounterValue(char counter[]) {
    mySerialPort.print(actualValueOf(counter));
}

void displayText(String text) {
  if (DEBUG) Serial.println("Received text to display: " + text);
  text = (text+(text.length() <= 4?"    ":"")).substring(0, 4); // Add spaces to strings less than 4 chars and substring for more than 4
  if (DEBUG) Serial.println("Printing text to display: " + text);
  mySerialPort.write(text.charAt(0));
  mySerialPort.write(text.charAt(1));
  mySerialPort.write(text.charAt(2));
  mySerialPort.write(text.charAt(3));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// Add and Subtract functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void add1To(char counter[], int* value) {
  int digit = 3;
  boolean finished = false;
  while (!finished && digit >= 0) {
    counter[digit] = counter[digit] + 1;
    if (counter[digit] > '9') {
      counter[digit] = '0';
      digit -= 1;
    } else {
      finished = true;
    }
  }
  
  displayCounterValue(counter);
  
  if (*value == 9999) {
    *value = 0;
  } else {
    *value = *value + 1;
  }
}

void subtract1To(char counter[], int* value) {
  int digit = 3;
  boolean finished = false;
  while (!finished && digit >= 0) {
    counter[digit] = counter[digit] - 1;
    if (counter[digit] < '0') {
      counter[digit] = '9';
      digit -= 1;
    } else {
      finished = true;
    }
  }
  
  displayCounterValue(counter);
  
  if (*value == 0) {
    *value = 9999;
  } else {
    *value = *value - 1;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
// Display brightness control
////////////////////////////////////////////////////////////////////////////////////////////////////

void highBright() {
  mySerialPort.write("z");
  mySerialPort.write((uint8_t)0); // Had to cast to uint8_t to distinguish from function Print::write(const char*)
};

void mediumBright() {
  mySerialPort.write("z");
  mySerialPort.write(31);
};

void lowBright() {
  mySerialPort.write("z");
  mySerialPort.write(255);
};

void displayOff() {
  mySerialPort.write("xxxx");
  mySerialPort.write("xxxx");
};

void setBrightLevel(int level) {
  // level can be between 0 and 8
  if (level == 8)
    level = 0;
  else
    level = (abs(level - 8) * 8) - 1;
  if (DEBUG) {Serial.print("Brightness level set to: ");Serial.println(level);}
  mySerialPort.write("z");
  mySerialPort.write(level);
};

// Displays dots control

void setDotsState(int state) {
  dotsLastState = state;
  sendDotsState();
};

void turnOffDot1() {
  dotsLastState = B11110111 & dotsLastState;
  sendDotsState();
};

void turnOnDot1() {
  dotsLastState = B00001000 | dotsLastState;
  sendDotsState();
};

void turnOffDot2() {
  dotsLastState = B11111011 & dotsLastState;
  sendDotsState();
};

void turnOnDot2() {
  dotsLastState = B00000100 | dotsLastState;
  sendDotsState();
};

void turnOffDot3() {
  dotsLastState = B11111101 & dotsLastState;
  sendDotsState();
};

void turnOnDot3() {
  dotsLastState = B00000010 | dotsLastState;
  sendDotsState();
};

void turnOffDot4() {
  dotsLastState = B11111110 & dotsLastState;
  sendDotsState();
};

void turnOnDot4() {
  dotsLastState = B00000001 | dotsLastState;
  sendDotsState();
};

void turnOffMiddleDots() {
  dotsLastState = B11101111 & dotsLastState;
  sendDotsState();
};

void turnOnMiddleDots() {
  dotsLastState = B00010000 | dotsLastState;
  sendDotsState();
};

void turnOffUpperDot() {
  dotsLastState = B11011111 & dotsLastState;
  sendDotsState();
};

void turnOnUpperDot() {
  dotsLastState = B00100000 | dotsLastState;
  sendDotsState();
};

void sendDotsState() {
  mySerialPort.write("w");
  mySerialPort.write(dotsLastState);
};

// Show "bulb" text in display
void displayBULB() {
  setDigitStatus(3, B01111100); // b
  setDigitStatus(2, B00011100); // u
  setDigitStatus(1, B00110000); // l
  setDigitStatus(0, B01111100); // b
}

// Control each element manually
void setDigitStatus(int digit, byte leds) {
  /*
    Segment Bit  LED
    *       7    not used
    G       6    middle horizontal
    F       5    top left vertical
    E       4    bottom left vertical
    D       3    bottom horizontal
    C       2    bottom right vertical
    B       1    top right vertical
    A       0    top horizontal

       A
    F     B
       G
    E     C
       D

    For example to turn on the 3 horizontal leds (A, G and D) on digit 0:
      mySerialPort.write(123);
      mySerialPort.write(B01001001);
  */
  if (digit >= 0 && digit <= 3) {
    mySerialPort.write(126-digit); // Access to individual digit
    mySerialPort.write(leds); // Set leds individually on/off
  }
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Auxiliary functions
////////////////////////////////////////////////////////////////////////////////////////////////////

// Casts a char[] (representing counter) to String
String actualValueOf(char counter[]) {
  return String(counter[0]) + String(counter[1]) + String(counter[2]) + String(counter[3]);
}

int toInt(char counter[]) {
  return (counter[0] - ASCII_NUMBER_POSITION) * 1000 + (counter[1] - ASCII_NUMBER_POSITION) * 100 + (counter[2] - ASCII_NUMBER_POSITION) * 10 + (counter[3] - ASCII_NUMBER_POSITION);
}

void printDebugData() {
  if (DEBUG) {
    Serial.println("-DEBUG-DATA-------------------------------");
    Serial.print(" * state: ");
    if (state == 0) {
      Serial.print("READING_PICS ");
    } else if (state == 1) {
      Serial.print("READING_INTERVAL ");
    } else if (state == 2) {
      Serial.print("TAKING_PICTURES ");
    }
    Serial.println(state, DEC);
    Serial.print(" * pics: "); Serial.println(pics);
    Serial.print(" * picsValue: ");Serial.println(picsValue, DEC);
    Serial.print(" * interval: "); Serial.println(interval);
    Serial.print(" * intervalValue: ");Serial.println(intervalValue, DEC);
    for (int i = 0 ; i < NUM_BUTTONS ; i++) {
      Serial.print(" * Button[");
      if (i == RIGHT) Serial.print("RIGHT]:");
      if (i == LEFT) Serial.print("LEFT]:");
      if (i == DOWN) Serial.print("DOWN]:");
      if (i == UP) Serial.print("UP]:");
      Serial.print(" actualState:"); Serial.print(Buttons[i].actualState);
      Serial.print(" previousState:"); Serial.print(Buttons[i].previousState);
      Serial.print(" fastForward:"); Serial.print(Buttons[i].fastForward);
      Serial.print(" previousMillis:"); Serial.print(Buttons[i].previousMillis);
      Serial.println("");
    }
    Serial.println("------------------------------------------");
  }
}
