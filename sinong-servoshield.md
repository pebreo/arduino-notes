
```
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo
Servo myservo3;  // create servo object to control a servo
Servo myservo4;  // create servo object to control a servo
int myservo_pin = 11;
int myservo_pin2 = 10;
int myservo_pin3 = 9;
int myservo_pin4 = 6;

int myservo_default_pos = 0;
int myservo_default_pos2 = 0;
int myservo_default_pos3 = 0;
int myservo_default_pos4 = 0;

int potpin = 0;
int potpin2 = 1;
int potpin3 = 2;
int potpin4 = 3;
int val;
int val2;
int val3;
int val4;


void setup() {
  myservo.attach(11);
  //myservo.write(7);
  
  myservo2.attach(10);
  //myservo2.write(7);
  
  myservo3.attach(9);
  //myservo3.write(7);
  
  myservo4.attach(6);
  //myservo4.write(7);
  //myservo.write(myservo_default_pos);
  /*
  myservo2.attach(myservo_pin2);
  myservo2.write(myservo_default_pos2);
  
  myservo3.attach(myservo_pin3);
  myservo3.write(myservo_default_pos3);
  
  myservo4.attach(myservo_pin4);
  myservo4.write(myservo_default_pos4);
  // put your setup code here, to run once:
  */
  Serial.begin(115200);
}

void loop() {
  // put you main code here, to run repeatedly:
  val = analogRead(potpin);
  val = map(val,1023,0,7,180);
    val2 = analogRead(potpin2);
  val2 = map(val2,1023,0,7,180);

    val3 = analogRead(potpin3);
  val3 = map(val3,1023,0,7,180);

    val4 = analogRead(potpin4);
  val4 = map(val4,1023,0,7,180);
  
  Serial.println("pot value:");
  Serial.println(val);
    Serial.println("pot value2:");
  Serial.println(val2);
    Serial.println("pot value3:");
  Serial.println(val3);
    Serial.println("pot value4:");\
  Serial.println(val4);

  myservo.write(val);
  myservo2.write(val2);
  myservo3.write(val3);
  myservo4.write(val4);
  /*
  myservo2.write(val2);
  myservo3.write(val3);
  myservo4.write(val4);*/
  delay(15);
}
```