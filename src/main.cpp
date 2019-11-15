#include <Arduino.h>
#include <open-celluloid.h>
#include <HardwareSerial.h>

//Serial.begin(115200);
//using namespace TMC2660_n;


  OpenCelluloid upuaut;  
  TimerOne  timer1;
  const int  testValue = 666;
  int incomingByte = 0; // for incoming serial data


void setup() {
 
  Serial.begin(115200);

	pinMode(enabPin, OUTPUT);
	pinMode(stepPin, OUTPUT);
	pinMode(chipSelect, OUTPUT);
	pinMode(dirPin, OUTPUT);
  pinMode(sensor, INPUT_PULLUP);
  pinMode(led, OUTPUT);


  digitalWrite(enabPin, LOW);
  digitalWrite(dirPin, LOW);
  digitalWrite(stepPin, LOW);
  digitalWrite(chipSelect, HIGH);
  SPI.begin();

	pinMode(misoSdo, INPUT_PULLUP);
  pinMode(mosiSdi, OUTPUT);
  pinMode(clk, OUTPUT);
//
  //upuaut.accelerated = true;
  upuaut.setup2130();
  upuaut.setupAccelStepper();
  upuaut.stepper.setCurrentPosition(0);
 
  //timer1.initialize(200);
  //timer1.attachInterrupt(triggerSensor);
  //Serial.flush();
 
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB port only
  // }
  Serial.println("??");
  Serial.println("This is the serial outside the class");
  Serial.println(Serial.peek());
  Serial.println(Serial.available());
  upuaut.state = auto_end;
  //upuaut.serialPrintln("This is a serial test");

  //upuaut.serialAvailable();

};

void loop() {

  // Serial.println("Loop");
  // if (Serial.available() > 0) {
  //   // read the incoming byte:
  //   incomingByte = Serial.read();

  //   // say what you got:
  //   Serial.print("I received: ");
  //   Serial.println(incomingByte, DEC);
  // }

  upuaut.stateSwitch();
  upuaut.serialTask();
}