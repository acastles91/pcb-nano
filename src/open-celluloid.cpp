#include "open-celluloid.h"



//_________________________________________________
void OpenCelluloid::setup2130() {
	

    //TMCStepper - simple !! Works with hardware constructor, TMCStepper library

    driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
    driver.toff(1);                 // Enables driver in software
    driver.tbl(1);
    driver.rms_current(900);        // Set motor RMS current
    driver.microsteps(4);          // Set microsteps to 1/16th

    driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
    driver.pwm_autoscale(true);     // Needed for stealthChop
    driver.semin(5);
    driver.semax(2);

};

//_________________________________________________
void OpenCelluloid::accelerate(){

  int speedTest = 500;
    
        for(int i = 0; (speedTest * 3) - i == 500; i++){

            digitalWrite(stepPin, HIGH);
            delayMicroseconds((speedTest * 3) - i);
            digitalWrite(stepPin, LOW);
            delayMicroseconds((speedTest * 3) - i);
        }
    
};

//_________________________________________________
void OpenCelluloid::moveMotor(){
	//while(true){
		//digitalWrite(enabPin, LOW);
		digitalWrite(stepPin, HIGH);
        delayMicroseconds(motorSpeed);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(motorSpeed);
		//Serial.println("Beep");
		//digitalWrite(enabPin, HIGH);

    //}

};

//_________________________________________________
void OpenCelluloid::moveMotorSlow() {
  oneFrameSlow();
};
//_________________________________________________
bool OpenCelluloid::isAccelerated(){
	return	accelerated;
};
//_________________________________________________
void OpenCelluloid::setupAccelStepper(){

    stepper.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
    stepper.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
    stepper.setEnablePin(enabPin);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
};
//_________________________________________________
void OpenCelluloid::stepperTimer(){
  if (stepCount > 0) {
    if (stepState == 0) {
      stepCount--;
    }
    digitalWrite(stepPin, stepState);
    stepState = !stepState;
  }
};
//_________________________________________________
void OpenCelluloid::homing(){

  digitalWrite(enabPin, LOW);
  digitalWrite(dirPin, LOW);
  int val = 0;
  bool gate_closed = true; //boolean related roughly to the shutter in front of the sensor
  bool center_not_found = true; //boolean related precisely to the sensor being at the end of the shutter
  byte gate[8]; //array used to make an average of the values; 0 means the shutter is in front of the sensor
  int sum_gate = 0;

  while (gate_closed == true && center_not_found == true) {
    for (int i = 0; i < 8; i ++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(50);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(50);
      val = !digitalRead(sensor);
      gate[i] = val;
    }
    for (int i = 0; i < 8; i ++) {
      sum_gate += gate[i];
      //Serial.println(gate[i]);
    }
    if (sum_gate == 0) {
      gate_closed = false;
      continue;
    } else {
      sum_gate = 0;
      continue;
    }
  }

  if (gate_closed == false) {
    //val = digitalRead(sensor);
    while (center_not_found == true) {
      //Serial.println("buscando el 8");
      for (int i = 0; i < 8; i ++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(50);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(50);
        val = !digitalRead(sensor);
        gate[i] = val;
      }
      for (int i = 0; i < 8; i ++) {
        sum_gate += gate[i];
        //Serial.println(gate[i]);
      }
      //Serial.println(sum_gate);
      if (sum_gate == 8) {
        center_not_found = false;
        continue;
      } else {
        sum_gate = 0;
        continue;
      }
    }
  }
  digitalWrite(dirPin, HIGH);
  gateOpen = true;
  moveMotorSlow();
};
//_________________________________________________
void OpenCelluloid::checkTrigger(){

  if(gateOpen){
   if(triggerCounter < 8){
      threshold[triggerCounter] = digitalRead(sensor);
      sum_threshold += threshold[triggerCounter];
    }
    else{
      triggerCounter = 0;    
    }
    if (sum_threshold == 8){
      sameState = false;
      sum_threshold = 0;
      gateOpen = false;
      shutterCounter += 1;
    }
    else if (sum_threshold > 8){
      sum_threshold = 0;
    }   
  }
  if (!gateOpen){
    triggerCounter = 0;
    if(digitalRead(sensor) == 1){
      gateOpen = true;
      shutterCounter +=1;
    }
  }
};

//_________________________________________________
void OpenCelluloid::testMove(){
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(500);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(500);
  checkTrigger();
  if(shutterCounter == 4){
    //Serial.println("Aqui");
    //Serial.println(shutterCounter);
    delay(1000);
    shutterCounter = 0;
  }
};

//_________________________________________________
void OpenCelluloid::readSensor(){
  serial->println(digitalRead(sensor));
}
void OpenCelluloid::oneFrame() {
  for (int i = 0; i < frameRatio ; i ++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(200);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(200);
  }
};

//_________________________________________________
void OpenCelluloid::oneFrameSlow() {
  for (int i = 0; i < frameRatio ; i ++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
};

//_________________________________________________
void OpenCelluloid::oneHundredFrames() {
  for (int i = 0; i < 100; i ++) {
    oneFrame();
  }
};

//_________________________________________________
void OpenCelluloid::serialFrames(){
    
  serial->write('0' + 0);
  delay(1000);
};

//_________________________________________________
void OpenCelluloid::triggerSensor(){
    if(digitalRead(sensor) == false){
    boolGate = true;
  }
  else{
    boolGate = false;
  }
  if(boolGate != boolState){
    triggerCounter += 1;
    boolState = boolGate;
  }
  if (triggerCounter == 4){
    Serial.write('0' + 0);
    triggerCounter = 0;
  }
};

//_________________________________________________
void OpenCelluloid::setSerial(Stream *streamObject){
  _streamRef = streamObject;
};
//_________________________________________________
void OpenCelluloid::serialPrintln(char *somePrintln){
  _streamRef->println(somePrintln);
};
//_________________________________________________
void OpenCelluloid::serialWrite(char *someWrite){
  _streamRef->write(someWrite);
};
//_________________________________________________
int OpenCelluloid::serialRead(void){
  _streamRef->read();
};
//_________________________________________________
int OpenCelluloid::serialAvailable(){
  _streamRef->available();
  _streamRef->println("Serial inside class is available");
};
//_________________________________________________
void OpenCelluloid::serialTask() {
  Serial.println("Begin of serial task");
  //serialPrintln("Begin of Serial Task");
  switch (state) {
    case auto_reset:
      homing();
      Serial.println("Homing executed");
      state = auto_end;
      //Serial.println(state);
      break;

    case start_moving_forward:
      if ( !isMoving) {
        homing();
        isMoving = true;
      } else {
        state = keep_moving;
        isMoving = true;
      }
      digitalWrite(dirPin, LOW);
      delayMicroseconds(10000);
      break;

    case start_moving_backward:
      if ( !isMoving) {
        homing();
        isMoving = true;
      } else {
        state = keep_moving;
        isMoving = true;
      }
      digitalWrite(dirPin, HIGH);
      delayMicroseconds(10000);
      break;

    case keep_moving:
      home_position = false;
      moveMotor();
      //serialFrames();
      state = keep_moving;
      break;

    case keep_moving_slow:
      home_position = false;
      moveMotorSlow();
      state = keep_moving_slow;
      break;

    case stoping:
      digitalWrite(stepPin, LOW);
      digitalWrite(enabPin, HIGH);
      state = auto_end;
      isMoving = false;
      digitalWrite(enabPin, LOW);
      break;

    case auto_end:
      home_position = false;
      break;

    case one_frame:
      //oneFrame();
      oneFrameSlow();
      state = auto_end;
      break;
      
    case loading:
      if ( !isMoving) {
        isMoving = true;
      } else {
        state = keep_moving_slow;
        isMoving = true;
      }
      digitalWrite(dirPin, LOW);
      break;
    case hundred_frames:
      oneHundredFrames();
      state = auto_end;
      break;
    
    case test:
      readSensor();
      break;
     
  }
};
//_________________________________________________
void OpenCelluloid::stateSwitch() {
    Serial.println("Begin of OpenCelluloid::stateSwitch()");
    //serialPrintln("Begin of OpenCelluloid::stateSwitch()");
    //serialPrintln(state);
    //Serial.println("Start state: " + upuaut.state);
    //while(!Serial){
    if (Serial.available() >= 0) {
        Serial.println(state);
        Serial.println("Serial available");
        uint8_t code = Serial.read();
        Serial.println(code);
        Serial.println("??");
        switch (code) {
        case 97:

            state = auto_reset;
            Serial.print("I received: ");
            Serial.println(code);
            break;
        case 98:
            state = start_moving_forward;
            Serial.print("I received: ");
            Serial.println(code);
            break;
        case 99:
            state = start_moving_backward;           
            Serial.print("I received: ");
            Serial.println(code);
            break;
        case 'd':
            state = stoping;
            Serial.print("I received: ");
            Serial.println(code);
            break;
        case 'e':
            state = one_frame;
            break;
        case 'f':
            state = hundred_frames;
            break;
        case 'g':
            state = loading;
            break;
        case 'h':
            state = test;
            break;
            
        case '0':
            state = stoping;
            break;
        case '1':
            state = start_moving_forward;
            break;
        case '2':
            state = start_moving_backward;
            break;
        case '4':
            state = auto_reset;
            break;
        }
        Serial.print("End state: ");
        Serial.println(state);
        }
        // else{
        //     Serial.println("Serial not available");
        // }
    };
