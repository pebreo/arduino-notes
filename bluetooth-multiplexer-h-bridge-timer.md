```c++
//main.cpp
//main.cpp
/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <TimerOne.h>
#include <Servo.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"

// Create the bluefruit object, either software serial...uncomment these lines

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

//  pins on 595 chip
const int enA = 5; // right
const int enB = 6; //left

const int pinA1 = 1; 
const int pinA2 = 2;

const int pinB1 = 3;
const int pinB2 = 4;


int SER_Pin = 2; //pin 14 on the 75HC595
int RCLK_Pin = 3; //pin 12 on the 75HC595
int SRCLK_Pin = 5; //pin 11 on the 75HC595

//How many of the shift registers – change this
#define number_of_74hc595s 1

//do not touch
#define numOfRegisterPins number_of_74hc595s * 8

boolean registers[numOfRegisterPins];

volatile bool up_pressed = false;
volatile bool down_pressed = false;
volatile bool left_pressed = false;
volatile bool right_pressed = false;

volatile bool one_pressed = false;
volatile bool one_released = false;
volatile bool two_pressed = false;
volatile bool two_released = false;
volatile bool three_pressed = false;
volatile bool three_released = false;
volatile bool four_pressed = false;
volatile bool four_released = false;

const int servoA_pin = 6;
const int servoB_pin = 9;

volatile float servoA_angle = 0;
const int servoA_min = 0;
const int servoA_max = 110;
volatile float servoB_angle = 0.0;
const int servoB_min = 0;
const int servoB_max = 180;

Servo servoA;  // create servo object to control a servo
Servo servoB;
int pos_A = 0;
int pos_B = 0;
void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  pinMode(SER_Pin, OUTPUT);
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);

  //reset all register pins
  clearRegisters();
  writeRegisters();
  setRegisterPin(enA, HIGH);
  setRegisterPin(enB, HIGH);
  //setRegisterPin(pinB1, HIGH);
  //setRegisterPin(pinB2, HIGH);
  writeRegisters(); //MUST BE CALLED TO DISPLAY CHANGES
  
  servoA.attach(6);
 
  servoB.attach(9);  
   /*
  servoA.write((int)servoA_min);

  
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);
  
  //digitalWrite(enA, HIGH);
  //digitalWrite(enB, HIGH);
  */
  //pinMode(LED_BUILTIN, OUTPUT);
  delay(500);
  // 20,000 microseconds -> 20 ms
  //Timer1.initialize(20000);
  //Timer1.attachInterrupt(addAngle);
  
  //Timer2.initialize(20000);
  //Timer2.attachInterrupt(subtractAngle);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}



/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  
  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    //handle_button(buttnum);
    
    if (pressed) {
      Serial.println(" pressed");
      if(buttnum == 5) {
          leftForward();
          rightForward();
      }
      if(buttnum == 6) {
          Serial.print("butt 5 - down");
          leftBackward();
          rightBackward();
      }
      if(buttnum == 7) {
          Serial.print("butt 5 - left");
          leftBackward();
          rightForward();

      }
      if(buttnum == 8) {
          Serial.print("butt 5 - right");
          leftForward();
          rightBackward();
      }
      if(buttnum == 1) {
          Serial.print("one");
           one_pressed = true;
           one_released = false;
           pos_A = pos_A - 30;
           if(pos_A >= servoA_max) {
            pos_A = servoA_max;
            }
           
      }
      if(buttnum == 2) {
          Serial.print("two");
          two_pressed = true;
          two_released = false;
           pos_A = pos_A + 30;
           if(pos_A >= servoA_max) {
            pos_A = servoA_max;
           }
      }

      if(buttnum == 3) {
          Serial.print("three");
          three_pressed = true;
          three_released = false;
          pos_B = pos_B - 30;
           if(pos_B >= servoB_max) {
            pos_B = servoB_max;
            }
          
      }
      if(buttnum == 4) {
          Serial.print("four");
          four_pressed = true;
          four_released = false;
          pos_B = pos_B + 30;
          if(pos_B >= servoB_max) {
            pos_B = servoB_max;
           }
        
      }
      
 
      
    } else {
      Serial.println(" released");
      
      Serial.print("stop");
      // right brake
      leftBrake();
      rightBrake();

      if(buttnum == 1) {
          Serial.print("one");
           one_pressed = false;
           one_released = true;
          //int a = (int)servoA_angle;
          servoA.write(pos_A);
          delay(15);

      }
      if(buttnum == 2) {
          Serial.print("two");
          two_pressed = false;
          two_released = true;
          //int a = (int)servoA_angle;
          servoA.write(pos_A);
          delay(15);
      }

      if(buttnum == 3) {
          Serial.print("three");
          three_pressed = false;
          three_released = true;
          servoB.write(pos_B);
          delay(15);
          
      }
      if(buttnum == 4) {
          Serial.print("four");
          four_pressed = false;
          four_released = true;
          servoB.write(pos_B);
          delay(15);
        
      }
      
      //handle_release_button(buttnum);
      Serial.println("servo A: ");
      Serial.println(servoA_angle);


      Serial.println("servo B: ");
      Serial.println(servoB_angle);
    }
  }

}


void onLight(void) {
  digitalWrite(LED_BUILTIN, HIGH);
}
void offLight(void) {
  digitalWrite(LED_BUILTIN, LOW);
}


void handle_button(uint8_t buttnum) {
  
   switch(buttnum)
  {

    case 5:
      Serial.println("up");
       leftForward();
       rightForward();
      break;
    case 6:
      Serial.println("down");
      leftBackward();
      rightBackward();
      break;
    case 7:
      Serial.println("left");
      leftBackward();
      rightForward();
      break;
    case 8:
      Serial.println("right");
      rightBackward();
      leftForward();
      break;
    case 1:
      Serial.println("one");
      one_pressed = true;
      one_released = false;
      //leftBrake();
      //rightBrake();
      break;
    case 2:
      Serial.println("two");
      two_pressed = true;
      two_released = false;
      //leftBrake();
      //rightBrake();
      break;
    case 3:
      Serial.println("three");
      three_pressed = true;
      three_released = false;
      //leftBrake();
      //rightBrake();
      break;
    case 4:
      Serial.println("four");
      four_pressed = true;
      four_released = false;
      //leftBrake();
      //rightBrake();
      break;
    
    default: 
      Serial.println("default");
      
      
  }
  //delay(10);
}


void handle_release_button(uint8_t buttnum)
{
   switch(buttnum)
  {

    case 5:
      Serial.println("up release");
      up_pressed = false;
      //offLight();
      break;
    case 6:
      Serial.println("down release");
      //leftBackward();
      //rightBackward();
      break;
    case 7:
      Serial.println("left release");
      //leftBackward();
      //rightForward();
      break;
    case 8:
      Serial.println("right release");
      //rightBackward();
      //leftForward();
      break;
    case 1:
      Serial.println("one release");
      one_pressed = false;
      one_released = true;
      
      //leftBrake();
      //rightBrake();
      break;
    case 2:
      Serial.println("two release");
      two_pressed = false;
      two_released = true;
      //leftBrake();
      //rightBrake();
      break;
    case 3:
      Serial.println("three release");
      three_pressed = false;
      three_released = true;
      //leftBrake();
      //rightBrake();
      break;
    case 4:
      Serial.println("four release");
      four_pressed = false;
      four_released = true;
      //leftBrake();
      //rightBrake();
      break;
    default: 
      Serial.println("default");
      
  }
  //delay(10);
}


void addAngle()
{

if(one_pressed == true)
 {
  
   servoA_angle = servoA_angle - 0.72;
   if(servoA_angle <= servoA_min) {
    servoA_angle = servoA_min;
   }
   //servoA.write((int)servoA_angle);
 }
 
 if(two_pressed == true)
 {
  // 180 deg every 5 seconds 
  
  servoA_angle = servoA_angle + 0.72;
  if(servoA_angle >= servoA_max) {
    servoA_angle = servoA_max;
  }
   //servoA.write((int)servoA_angle);
 }
 
  if(four_pressed == true)
 {
  
   servoB_angle = servoB_angle + 0.72;
   if(servoB_angle >= servoB_max) {
    servoB_angle = servoB_max;
    }
    
 }

  if(three_pressed == true)
 {
  
   servoB_angle = servoB_angle - 0.72;
   if(servoB_angle <= servoB_min) {
    servoB_angle = servoB_min;
   }
 }

 
 /*
 else{
   the_number = 0;
 }*/
}



void rightForward()
{
  //clearRegisters();
    setRegisterPin(enA, HIGH);
  setRegisterPin(enB, HIGH);
  setRegisterPin(pinA1, LOW);
  setRegisterPin(pinA2, HIGH);
  writeRegisters(); //MUST BE CALLED TO DISPLAY CHANGES
}


void rightBackward()
{
  //clearRegisters();
    setRegisterPin(enA, HIGH);
  setRegisterPin(enB, HIGH);
  setRegisterPin(pinA1, HIGH);
  setRegisterPin(pinA2, LOW);
  writeRegisters(); //MUST BE CALLED TO DISPLAY CHANGES
}


void leftForward()
{
   //clearRegisters();
     setRegisterPin(enA, HIGH);
  setRegisterPin(enB, HIGH);
  
   setRegisterPin(pinB1, HIGH);
   setRegisterPin(pinB2, LOW);
   writeRegisters(); //MUST BE CALLED TO DISPLAY CHANGES
   
}

void leftBackward()
{
  
     setRegisterPin(enA, HIGH);
  setRegisterPin(enB, HIGH);
  
   setRegisterPin(pinB1, LOW);
   setRegisterPin(pinB2, HIGH);
   writeRegisters(); //MUST BE CALLED TO DISPLAY CHANGES
  
}

void leftBrake()
{
  
   //digitalWrite(pinB1, HIGH);
   //digitalWrite(pinB2, HIGH);
   setRegisterPin(pinB1, HIGH);
   setRegisterPin(pinB2, HIGH);
    writeRegisters(); //MUST BE CALLED TO DISPLAY CHANGES
}

void rightBrake()
{
   //digitalWrite(pinA1, HIGH);
   //digitalWrite(pinA2, HIGH);
   setRegisterPin(pinA1, HIGH);
   setRegisterPin(pinA2, HIGH);
    writeRegisters(); //MUST BE CALLED TO DISPLAY CHANGES
  
}


//set all register pins to LOW
void clearRegisters()
{ 
  for(int i = numOfRegisterPins - 1; i >= 0; i--) {
        registers[i] = LOW;
    }
}

//Set and display registers
//Only call AFTER all values are set how you would like (slow otherwise)
void writeRegisters()
{

    digitalWrite(RCLK_Pin, LOW);

    for(int i = numOfRegisterPins - 1; i >= 0; i--) {
        digitalWrite(SRCLK_Pin, LOW);

        int val = registers[i];

        digitalWrite(SER_Pin, val);
        digitalWrite(SRCLK_Pin, HIGH);
    }
    digitalWrite(RCLK_Pin, HIGH);
}

//set an individual pin HIGH or LOW
void setRegisterPin(int index, int value)
{
    registers[index] = value;
}
```