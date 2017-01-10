//select analog pin 2 for potentiometer
int potPin1 = 1;
int potPin2 = 3;
//potVal stores the value of the voltage on potPin
int potVal1 = 0;
int potVal2 = 0;

//the buffer that is written as a serial message
byte buf[4];
// for timing
unsigned long lastSignal = 0;
unsigned long interval = 200;  //milli seconds

void setup() {
  // this initializes the serial connection via usb, baud rate 9600
  Serial.begin(9600);
}

void loop() {
  while(millis() - lastSignal > interval)
    {
        lastSignal += interval;
        // read the voltage at potPin
        potVal1 = analogRead(potPin1);
       // potVal1 = (int)(((float)potVal1/512 - 1.0)*1000);
        
        potVal2 = analogRead(potPin2);
        //potVal2 = (int)(((float)potVal2/512 - 1.0)*1000);
 
        // assemble the buffer
        buf[0] = lowByte(potVal1);
        buf[1] = highByte(potVal1);
 
        buf[2] = lowByte(potVal2);
        buf[3] = highByte(potVal2);

        Serial.write(buf ,4);
    }
}

