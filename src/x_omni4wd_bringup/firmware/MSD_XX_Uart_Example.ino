#include <SoftwareSerial.h>
SoftwareSerial softSerial(12, 13); //Define PIN12 & PIN13 as software UART

const int ledPin =     10; //LED_BUILTIN;// the number of the LED pin

// Variables will change:
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 10000;           // interval at which to blink (milliseconds)

void MSD_UartReadDriver(){
   softSerial.println("Hello: MSD ");   //Have to send some no meaning bytes to fix a bug of Software uart. 
   softSerial.println("N0 ?");          //Find exited driver  in the Uart_Network by send a requesting  broadcat command
   delay(2000);                         //Waitting 2s to get feedback from Drivers. 
   if (softSerial.available()) {
    Serial.write(softSerial.read());    //Prinitting to Consol. 
  }
}

void setup() {
  softSerial.begin(19200);   // set the data rate for the SoftwareSerial port
  Serial.begin(19200); // opens serial port, sets data rate to 115200 bps
  delay(1000);
  
  pinMode(10, OUTPUT);          // sets the digital pin 13 as output
  MSD_UartReadDriver();
}




void loop() {
  unsigned long currentMillis = millis();
  if (softSerial.available()) {
    Serial.write(softSerial.read());
   // Serial.write("hello");
  }
 
  if (currentMillis - previousMillis >= interval) { //delay 4S
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
      softSerial.println("N1 p100 v50 a2000 O G3");  //send to driver Add =1 (N1), Vị trí tuyet doi p=100 rad, Vận tốc v=50rad/s và gia tốc a =2000 rad/s^2.  
    } 
    else 
    {
      ledState = LOW;
       softSerial.println("N1  p0 v100 a5000 O G3"); //All motor go to 0 again. 
    }
  }
}
