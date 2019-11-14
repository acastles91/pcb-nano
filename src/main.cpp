#include <Arduino.h>
#include <open-celluloid.h>

//Serial.begin(115200);
//using namespace TMC2660_n;


  OpenCelluloid upuaut;  
  TimerOne  timer1;
  const int  testValue = 666;

//   shared_ptr<OpenCelluloid> upuaut(new OpenCelluloid);


void triggerSensor(){
    if(digitalRead(sensor) == false){
    upuaut.boolGate = true;
  }
  else{
    upuaut.boolGate = false;
  }
  if(upuaut.boolGate != upuaut.boolState){
    upuaut.triggerCounter += 1;
    upuaut.boolState = upuaut.boolGate;
  }
  if (upuaut.triggerCounter == 4){
    upuaut.serial->write('0' + 0);
    upuaut.triggerCounter = 0;
  }
};



void setup() {
  
  upuaut.setupSerial();

  while(!upuaut.serial);
	upuaut.serial->println("\nStart...");

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

  upuaut.state = start_moving_forward;

  
  timer1.initialize(200);
  upuaut.serial->println("Timer initialized");
  timer1.attachInterrupt(triggerSensor);
  upuaut.serial->println("End of setup");

};

void loop() {

  upuaut.stateSwitch();
  upuaut.serialTask();
};