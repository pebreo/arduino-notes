```cpp


#include <IRremote.h>

int RECV_PIN = 7;

IRrecv irrecv(RECV_PIN);

decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {
  if (irrecv.decode(&results)) {
    //Serial.println(results.value, HEX);
    handle_codes();
    irrecv.resume(); // Receive the next value
  }
  delay(100);
}

void handle_codes()
{
  switch(results.value)
  {

    case 0x1E108:
      Serial.println("up");
      break;
    case 0x9E108:
      Serial.println("left");
      break;
    case 0x5E108:
      Serial.println("down");
      break;
    case 0xDE108:
      Serial.println("right");
      break;
    case 0x3E108:
      Serial.println("center");
      break;
    default: 
      Serial.println(results.value, HEX);
  }
  delay(500);
}
```


```