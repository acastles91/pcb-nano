#include "open-celluloid.h"


void OpenCelluloid::setupSerial(){

    serial->begin(115200);
};

void OpenCelluloid::setup2130() {
	
	//Software SPI
    // driver.begin(); 			// Initiate pins and registeries
	// driver.rms_current(600); 	// Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
	// driver.stealthChop(1); 	// Enable extremely quiet stepping
	
	// digitalWrite(enabPin, LOW);

	// Serial.print("DRV_STATUS=0b");
	// Serial.println(driver.DRV_STATUS(), BIN);

    //
    //StallGuard

    // driver.push();
    // driver.toff(3);
    // driver.tbl(1);
    // driver.hysteresis_start(4);
    // driver.hysteresis_end(-2);
    // driver.rms_current(600); // mA
    // driver.microsteps(16);
    // driver.diag1_stall(1);
    // driver.diag1_active_high(1);
    // driver.coolstep_min_speed(0xFFFFF); // 20bit max
    // driver.THIGH(0);
    // driver.semin(5);
    // driver.semax(2);
    // driver.sedn(0b01);
    // driver.sg_stall_value(STALL_VALUE);

    // digitalWrite(enabPin, LOW);

    //Accelstepper

    //pinMode(chipSelect, OUTPUT);
    //digitalWrite(chipSelect, HIGH);
    // driver.begin();             // Initiate pins and registeries
    // driver.rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    // driver.stealthChop(1);      // Enable extremely quiet stepping
    // driver.stealth_autoscale(1);
    // driver.microsteps(16);

    //TMCStepper - simple !! Works with hardware constructor, TMCStepper library

    driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
    driver.toff(1);                 // Enables driver in software
    driver.tbl(1);
    driver.rms_current(900);        // Set motor RMS current
    driver.microsteps(4);          // Set microsteps to 1/16th

    driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
//driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
    driver.pwm_autoscale(true);     // Needed for stealthChop
    //driver.irun(31);
    //driver.ihold(25);
    driver.semin(5);
    driver.semax(2);
    //TMCStepper - StallGuard

    // driver.begin();
    // driver.toff(4);
    // driver.blank_time(24);
    // driver.rms_current(400); // mA
    // driver.microsteps(2);
    // driver.TCOOLTHRS(0xFFFFF); // 20bit max
    // driver.THIGH(0);
    // driver.semin(5);
    // driver.semax(2);
    // driver.sedn(0b01);
    // driver.sgt(STALL_VALUE);




};

//  void OpenCelluloid::setup2660(){

// //   driver.begin();
// //   driver.toff(4);
// //   driver.blank_time(12);
// //   driver.rms_current(1000); // mA
// //   driver.microsteps(8);
// //   driver.sfilt(true); // Improves SG readout.
// //   driver.rdsel(0b01);
// //   //driver.semin(2);
// //   //driver.semax(10);
// //   driver.sedn(0b01);
// //   driver.sgt(STALL_VALUE);



// };
// OpenCelluloid::Opencelluloid(HardwareSerial *serial1){
//     _HardSerial = serial1;
// }
void OpenCelluloid::accelerate(){

  //bool test = false;
  int speedTest = 500;
    
    if (!test){
        for(int i = 0; (speedTest * 3) - i == 500; i++){

            digitalWrite(stepPin, HIGH);
            delayMicroseconds((speedTest * 3) - i);
            digitalWrite(stepPin, LOW);
            delayMicroseconds((speedTest * 3) - i);
        }
        //test = true;
    }


};

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
void OpenCelluloid::moveMotorSlow() {
  oneFrameSlow();
};


bool OpenCelluloid::isAccelerated(){
	return	accelerated;
};

void OpenCelluloid::setupAccelStepper(){

    stepper.setMaxSpeed(50*steps_per_mm); // 100mm/s @ 80 steps/mm
    stepper.setAcceleration(1000*steps_per_mm); // 2000mm/s^2
    stepper.setEnablePin(enabPin);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
};

void OpenCelluloid::stepperTimer(){
  if (stepCount > 0) {
    if (stepState == 0) {
      stepCount--;
    }
    digitalWrite(stepPin, stepState);
    stepState = !stepState;
  }
};

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
    //Serial.println(sum_gate);

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

void OpenCelluloid::oneFrameSlow() {
  for (int i = 0; i < frameRatio ; i ++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
};

void OpenCelluloid::oneHundredFrames() {
  for (int i = 0; i < 100; i ++) {
    oneFrame();
  }
};

void OpenCelluloid::serialFrames(){
    
  serial->write('0' + 0);
  delay(1000);
};

void OpenCelluloid::serialTask() {

  switch (state) {
    case auto_reset:
      homing();
      //Serial.println("Homing executed2");
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
      //testMove();
      readSensor();
      break;
     
  }
};

// void OpenCelluloid::serialTask() {

//   switch (upuaut.state) {
//     case auto_reset:
//       upuaut.homing();
//       //Serial.println("Homing executed2");
//       upuaut.state = auto_end;
//       //Serial.println(state);
//       break;

//     case start_moving_forward:
//       if ( !upuaut.isMoving) {
//         upuaut.homing();
//         upuaut.isMoving = true;
//       } else {
//         upuaut.state = keep_moving;
//         upuaut.isMoving = true;
//       }
//       digitalWrite(dirPin, LOW);
//       delayMicroseconds(10000);
//       break;

//     case start_moving_backward:
//       if ( !upuaut.isMoving) {
//         upuaut.homing();
//         upuaut.isMoving = true;
//       } else {
//         upuaut.state = keep_moving;
//         upuaut.isMoving = true;
//       }
//       digitalWrite(dirPin, HIGH);
//       delayMicroseconds(10000);
//       break;

//     case keep_moving:
//       upuaut.home_position = false;
//       upuaut.moveMotor();
//       //serialFrames();
//       upuaut.state = keep_moving;
//       break;

//     case keep_moving_slow:
//       upuaut.home_position = false;
//       upuaut.moveMotorSlow();
//       upuaut.state = keep_moving_slow;
//       break;

//     case stoping:
//       digitalWrite(stepPin, LOW);
//       digitalWrite(enabPin, HIGH);
//       upuaut.state = auto_end;
//       upuaut.isMoving = false;
//       digitalWrite(enabPin, LOW);
//       break;

//     case auto_end:
//       upuaut.home_position = false;
//       break;

//     case one_frame:
//       //oneFrame();
//       upuaut.oneFrameSlow();
//       upuaut.state = auto_end;
//       break;
      
//     case loading:
//       if ( !upuaut.isMoving) {
//         upuaut.isMoving = true;
//       } else {
//         upuaut.state = keep_moving_slow;
//         upuaut.isMoving = true;
//       }
//       digitalWrite(dirPin, LOW);
//       break;
//     case hundred_frames:
//       upuaut.oneHundredFrames();
//       upuaut.state = auto_end;
//       break;
    
//     case test:
//       //testMove();
//       upuaut.readSensor();
//       break;
     
//   }
// };

void OpenCelluloid::stateSwitch() {
    serial->println("Begin of OpenCelluloid::stateSwitch()");
    serial->println("Serial works, doesn't seem to read variables from Upuaut");
    serial->println(state);
    //Serial.println("Start state: " + upuaut.state);
    if (serial->available()) {
        serial->println("Serial available");
        uint8_t code = serial->read();
        //Serial.println(code);
        switch (code) {
        case 'a':
            state = auto_reset;
            break;
        case 'b':
            state = start_moving_forward;
            break;
        case 'c':
           state = start_moving_backward;
            break;
        case 'd':
            state = stoping;
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
        serial->println("End state: " + state);
        }
        else{
            serial->println("Serial not available");
        }

    };


// void OpenCelluloid::stateSwitch() {
//     Serial.println("Begin of stateSwitch()");
//     Serial.println("Serial works, doesn't seem to read variables from Upuaut");
//     Serial.println("Printing universal char: " + upuaut.state);
//     //Serial.println("Start state: " + upuaut.state);
//     if (Serial.available()) {
//         Serial.println("Serial available");
//         uint8_t code = Serial.read();
//         //Serial.println(code);
//         switch (code) {
//         case 'a':
//             upuaut.state = auto_reset;
//             break;
//         case 'b':
//             upuaut.state = start_moving_forward;
//             break;
//         case 'c':
//             upuaut.state = start_moving_backward;
//             break;
//         case 'd':
//             upuaut.state = stoping;
//             break;
//         case 'e':
//             upuaut.state = one_frame;
//             break;
//         case 'f':
//             upuaut.state = hundred_frames;
//             break;
//         case 'g':
//             upuaut.state = loading;
//             break;
//         case 'h':
//             upuaut.state = test;
//             break;
            
//         case '0':
//             upuaut.state = stoping;
//             break;
//             case '1':
//             upuaut.state = start_moving_forward;
//             break;
//             case '2':
//             upuaut.state = start_moving_backward;
//             break;
//             case '4':
//             upuaut.state = auto_reset;
//             break;
//         }
//         Serial.println("End state: " + upuaut.state);

//     }
// };


//unsigned long forwardDelay = 0;
//int forwardInterval = 10000

// static void OpenCelluloid::triggerSensor(){
//     if(digitalRead(sensor) == false){
//     boolGate = true;
//   }
//   else{
//     boolGate = false;
//   }
//   if(boolGate != boolState){
//     triggerCounter += 1;
//     boolState = boolGate;
//   }
//   if (triggerCounter == 4){
//     Serial.write('0' + 0);
//     triggerCounter = 0;
//   }
// };




