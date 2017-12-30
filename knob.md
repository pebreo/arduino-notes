
knob
https://www.arduino.cc/en/Tutorial/Knob

```avr
#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}
```


```avr


#include <Servo.h>

Servo myservo;
int potpin = 0;
int val;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // servo C
  myservo.attach(7);
  myservo.write(90);
}

void loop() {
  // put you main code here, to run repeatedly:
  val = analogRead(potpin);
  //val = map(val,1023,0,0,180);
  // 90 to 180  - 513 to 1023
  // 0 to 90 - 0 to 512

  // 512 to 0 - 91 to 180
  if(val <= 512) {
    val = map(val,512,0,91,180);
  }
  // 1023 to 513  - 0 to 90
  if(val > 512) {
    val = map(val,1023,513,0,90);
  }
  
  Serial.println("pot value:");
  Serial.println(val);
  myservo.write(val);
  delay(15);
}
```