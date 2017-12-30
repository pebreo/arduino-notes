```cpp
int enA = 2; // right
int enB = 5; //left

int pinA1 = 8; 
int pinA2 = 9;

int pinB1 = 10;
int pinB2 = 11;

void setup() 
{ 
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(pinA1, OUTPUT);
  pinMode(pinA2, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinB2, OUTPUT);
  
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);

}

void rightForward()
{
  digitalWrite(pinA1, LOW);
  digitalWrite(pinA2, HIGH);
}


void rightBackward()
{
  digitalWrite(pinA1, HIGH);
  digitalWrite(pinA2, LOW);
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


void demoLeft() 
{
  digitalWrite(pinB1, HIGH);
  digitalWrite(pinB2, LOW);

  delay(5000);
  // turn off motors
  digitalWrite(pinB1, HIGH);
  digitalWrite(pinB2, HIGH);
}

void demoRight() 
{
  digitalWrite(enA, HIGH);
  digitalWrite(pinA1, LOW);
  digitalWrite(pinA2, HIGH);

  delay(5000);
  digitalWrite(pinA1, HIGH);
  digitalWrite(pinA2, HIGH);

}

void loop() 
{ 
  // uncomment to test
  //demoLeft();
  //delay(3000);
  //demoRight();
}


```