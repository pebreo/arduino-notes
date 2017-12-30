http://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/

http://www.instructables.com/id/Arduino-Modules-L298N-Dual-H-Bridge-Motor-Controll/

```cpp



#include <IRremote.h>

int enA = 2; // right
int enB = 5; //left

int pinA1 = 8; 
int pinA2 = 9;

int pinB1 = 10;
int pinB2 = 11;

int RECV_PIN = 7; // 5 volts power to IR sensor



IRrecv irrecv(RECV_PIN);

decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);
  
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
}

void loop() {
  if (irrecv.decode(&results)) {
    //Serial.println(results.value, HEX);
    handle_codes();
    irrecv.resume(); // Receive the next value
  }
  delay(10);
}

void rightForward()
{
  digitalWrite(pinA1, LOW);
  digitalWrite(pinA2, HIGH);
  delay(1);
}


void rightBackward()
{
  digitalWrite(pinA1, HIGH);
  digitalWrite(pinA2, LOW);
  delay(1);
}

void leftForward()
{
   digitalWrite(pinB1, HIGH);
   digitalWrite(pinB2, LOW);
   delay(1);
}

void leftBackward()
{
   digitalWrite(pinB1, LOW);
   digitalWrite(pinB2, HIGH);
   delay(1);
}

void leftBrake()
{
  
   digitalWrite(pinB1, HIGH);
   digitalWrite(pinB2, HIGH);
   delay(1);  
}

void rightBrake()
{
   digitalWrite(pinA1, HIGH);
   digitalWrite(pinA2, HIGH);
   delay(1);  
  
}


void handle_codes()
{
  switch(results.value)
  {

    case 0x1E108:
      Serial.println("up");
      rightForward();
      leftForward();
      break;
    case 0x9E108:
      Serial.println("down");
      leftBackward();
      rightBackward();
      break;
    case 0x5E108:
      Serial.println("left");
      leftBackward();
      rightForward();
      break;
    case 0xDE108:
      Serial.println("right");
      rightBackward();
      leftForward();
      break;
    case 0x3E108:
      Serial.println("center");
      leftBrake();
      rightBrake();
      break;
    default: 
      Serial.println(results.value, HEX);
  }
  delay(10);
}


```