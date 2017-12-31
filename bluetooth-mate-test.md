
```cpp
// in the terminal
// screen /dev/tty.FireFly-E808-SPP 9600

#include <SoftwareSerial.h>  
int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

//volatile char val;
void setup()
{
  Serial.begin(9600);  // Begin the serial monitor at 9600bps

  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600
}

void loop()
{
  if(bluetooth.available())  // If the bluetooth sent any characters
  {
    // Send any characters the bluetooth prints to the serial monitor
    char val = bluetooth.read();
    Serial.print(val);
    // works
    //Serial.print((char)bluetooth.read());  
  }
  if(Serial.available())  // If stuff was typed in the serial monitor
  {
    // Send any characters the Serial monitor prints to the bluetooth
    bluetooth.print((char)Serial.read());
  }
  // and loop forever and ever!
}
```


```cpp

// /dev/tty.FireFly-E808-SPP
// screen /dev/tty.FireFly-E808-SPP 9600
#include <SoftwareSerial.h>  
int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3

// DOES NOT WORK FOR HARDWARE SERIAL PINOUTS. 
// YOU NEED BLUETOOTH MODULES HC-05/HC-06 FOR THIS TO WORK
//int bluetoothTx = 1;  // TX-O pin of bluetooth mate, Arduino D2
//int bluetoothRx = 0;  // RX-I pin of bluetooth mate, Arduino D3

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

char val; // variable to receive data from the serial port

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);       // start serial communication at 9600bps
  bluetooth.begin(19200);  // Start bluetooth serial at 9600
}

void loop() {

  if( bluetooth.available() )       // if data is available to read
  {
    val = bluetooth.read();         // read it and store it in 'val'
    Serial.write(bluetooth.read());
  }
  if( val == 'H' )               // if 'H' was received
  {
    digitalWrite(LED_BUILTIN, HIGH);  // turn ON the LED
  } else { 
    digitalWrite(LED_BUILTIN, LOW);   // otherwise turn it OFF
  }
  delay(100);                    // wait 100ms for next reading
} 
```