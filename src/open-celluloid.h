#pragma once


// #define startStop   2
// #define stepPin     3
// #define dirPin      4
// #define channel6    5
// #define channelB    6
// #define enabPin     7
// #define dirSwitch   8
// #define soundTrig   9
// #define chipSelect  10
// #define mosiSdi     11
// #define misoSdo     12
// #define clk         13
// #define led         A3
// #define potSpeed    A4
// #define encoButton  A5
// #define sensor      A7

// #define auto_reset              'a'
// #define start_moving_forward    'b'
// #define start_moving_backward   'c'
// #define keep_moving             'd'
// #define stoping                 'e'
// #define auto_end                'f'
// #define one_frame               'g'
// #define hundred_frames          'h'
// #define loading                 'i'
// #define keep_moving_slow        'j'
// #define test                    'k'


#define auto_reset              0
#define start_moving_forward    1
#define start_moving_backward   2
#define keep_moving             3
#define stoping                 4
#define auto_end                5
#define one_frame               6
#define hundred_frames          7
#define loading                 8
#define keep_moving_slow        9
#define test                    10

const int startStop = 2;
const int stepPin = 3;
const int dirPin = 4;
const int channel6 = 5;
const int channelB = 6;
const int enabPin = 7;
const int dirSwitch = 8;
const int soundTrig = 9;
const int chipSelect = 10;
const int mosiSdi = 11;
const int misoSdo = 12;
const int clk = 13;


#define led  A3
#define potSpeed  A4
#define encoButton  A5
#define sensor  A7

// #define STEP_PORT     3
// #define STEP_BIT_POS  3

#define STALL_VALUE 15
#define R_SENSE 0.1 // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

                      //BOB = 0.3
 

#include <Arduino.h>
//#include <TMC2130Stepper.h>
//#include <TMC2130Stepper_REGDEFS.h>
#include <TMCStepper.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <TimerOne.h>
#include <HardwareSerial.h>
#include <Stream.h>

//HardwareSerial Serial;



class OpenCelluloid{

    public:

        //OpenCelluloid(HardwareSerial *serial1) : ;
        TMC2130Stepper driver = TMC2130Stepper(chipSelect, R_SENSE); // hardware SPI
        AccelStepper stepper = AccelStepper(stepper.DRIVER, stepPin, dirPin);

        
        HardwareSerial *serial;
        //HardwareSerial &serial1 = Serial;
        //TimerOne timer1;
        
        // bool accelerated = false;
        // int motorSpeed = 250;
        // const uint32_t steps_per_mm = 80;
        bool isMoving = false;
        bool accelerated = false;
        int motorSpeed = 200;
        const uint32_t steps_per_mm = 80;

        const int frameRatio = 1600 / 2.55;
        const int sensorThreshold = 300;
        int lastSensorState = 0;
        uint8_t sensorState = 0; //
        uint8_t stepState = 0; //

        //trigger
        volatile bool gateOpen;
        volatile uint8_t trigger = 0;
        volatile uint8_t triggerCounter = 0;
        volatile bool sameState = true;
        volatile byte threshold[8];
        volatile uint8_t sum_threshold = 0;
        volatile uint8_t gate;
        volatile uint8_t gatePrevious;
        volatile uint8_t shutterCounter = 0;
        volatile bool boolGate;
        volatile bool boolState = !digitalRead(sensor);

        volatile bool home_position = false;
        volatile char state = auto_reset;

        

        uint8_t doFullRotation = 0;
        int stepCount = 10000;

        // const int led = A3;
        // const int potSpeed = A4;
        // const int encoButton = A5;
        // const int sensor = A7;

        //TMC2130Stepper driver  = TMC2130Stepper(enabPin, dirPin, stepPin, chipSelect, mosiSdi, misoSdo, clk);
        //TMC2130Stepper driver  = TMC2130Stepper(chipSelect, mosiSdi, misoSdo, clk);
        //TMC2130Stepper driver = TMC2130Stepper(chipSelect);
        //TMC2130Stepper driver = TMC2130Stepper(enabPin, dirPin, stepPin, chipSelect, mosiSdi, misoSdo, clk);
        //TMC2660Stepper  driver = TMC2660Stepper(chipSelect, R_SENSE); //R_SENSE in BOB is 0.3
        //TMC2130Stepper driver = TMC2130Stepper(chipSelect, mosiSdi, misoSdo, clk);
        //TMC2130Stepper driver = TMC2130Stepper(EN_PIN, DIR_PIN, STEP_PIN, CS_PIN);
        //TMCStepper
        // TMC2130Stepper(uint16_t pinCS, float RS = default_RS, int8_t link_index = -1);
		// TMC2130Stepper(uint16_t pinCS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK, int8_t link_index = -1);
		// TMC2130Stepper(uint16_t pinCS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK, int8_t link_index = -1);
        //TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
        //TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

        void accelerate();
        void moveMotor();
        void moveMotorSlow();
        void setup2130();
        //void setup2660();
        void setupAccelStepper();
        bool isAccelerated();
        void stepperTimer();
        void homing();
        void checkTrigger();
        //static void triggerSensor();
        void testMove();
        void readSensor();
        void oneFrame();
        void oneFrameSlow();
        void oneHundredFrames();
        void serialFrames();
        void serialTask();
        void stateSwitch();
        void triggerSensor();

        void setSerial(Stream *streamObject);
        void serialPrintln(char *somePrintln);
        void serialWrite(char *someWrite);
        int serialRead(void);
        int serialAvailable(void);

    private:

        Stream *_streamRef;

};