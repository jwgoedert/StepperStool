// random stepper motion sketch
// mc 2014-04-07

// include built in AVR delay functions
#include <util/delay.h>

// stepper driver control pins
// if there is an enable pin, uncomment lines as necessary
#define DIR 12
#define STEP 13
#define EN 11

#define GEARATIO 72

// stepper motor specs
#define DEGREES_PER_STEP   1.8
#define DRIVER_MICROSTEPS  2

// minimum delay between steps (lower means faster)
// defines fastest speed of the stepper
#define MIN_US_PER_STEP  600
// maximum delay between steps (lower is faster)
// defines the slowest speed of the stepper
#define MAX_US_PER_STEP  900

// rotation limits in degrees. positive in in DIR = HIGH direction
#define MIN_ROTATION  -90
#define MAX_ROTATION   90
// smallest allowed rotation in a single spurt (degrees)
#define MIN_SINGLE_ROTATION 15

// microsend delay between signal pulses for sending data to stepper driver
// value from datasheet
#define DRIVER_US_DATA_DELAY  2

// global variables
// keep track of current step and limits
// step limits are const because they will not change
const int16_t minRotSteps = MIN_ROTATION / DEGREES_PER_STEP * DRIVER_MICROSTEPS * GEARATIO;
const int16_t maxRotSteps = MAX_ROTATION / DEGREES_PER_STEP * DRIVER_MICROSTEPS * GEARATIO;
const int16_t minSingleRot = MIN_SINGLE_ROTATION / DEGREES_PER_STEP * DRIVER_MICROSTEPS * GEARATIO;
int16_t currentRot;
int8_t dir;

void setup(void) {
  // enable serial
  Serial.begin(19200);
  // wait for serial input before running sketch
  while(!Serial.available());
  Serial.read();
  
  // display parameters
  Serial.print("minrot: "); Serial.println(minRotSteps);
  Serial.print("maxrot: "); Serial.println(maxRotSteps);
  Serial.print("minsingt: "); Serial.println(minSingleRot);
  
  //enables pins on board to communicate
  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);

  // set the direction pin to an output
  pinMode(DIR, OUTPUT);
  // pull direction high to start
  digitalWrite(DIR, HIGH);
  // step is an output and set it to low initially
  pinMode(STEP, OUTPUT);
  digitalWrite(STEP, LOW);
  // if there's an enable pin, pull it low to enable stepper
  // check with datasheet to ensure it's an active low pin
  digitalWrite(EN, LOW);



  // get the step variables ready
  // rotation is zeroed at on position
  currentRot = 0;
  // current direction - 1 for DIR=HIGH, -1 for DIR=LOW
  dir=1;
}

void loop(void) {
  // generate a random number of steps to take
  // keep generating until a non-impossible movement is generated
  int8_t dir;
  int16_t steps;
  
  do {
    // pick a direction to go
    // cast random to an int8 because random returns a int32
    // this generates either a 0 or 1 (upper bound is non-inclusive)
    dir = (int8_t)(random(0,2));
    if (dir == 0) {
      dir = -1;
    }
    Serial.print("d: "); Serial.println(dir); 
    
    steps =(int16_t)( random(minSingleRot, maxRotSteps));
 
    Serial.print("g: "); Serial.println(steps); 
    Serial.print("new: "); Serial.println(currentRot + (dir * steps));
  } while( currentRot + (dir * steps ) >= maxRotSteps ||
           currentRot + (dir * steps ) <= minRotSteps );
  
  setDirection(dir);
  Serial.print("steps: "); Serial.println(steps);
  // generate a random number to decide motor speed
  int16_t us = (int16_t)(random(MIN_US_PER_STEP, MAX_US_PER_STEP));
  Serial.print("speed: "); Serial.println(us);
  // take that random number of steps and do them
  // do a decrementing for loop because they're a bit quicker on avr
  for (int16_t s = steps; s>0; s--) {
  // step and delay a few microseconds
    step();
    _delay_us(us);
  }

  // update the rotation tracking
  currentRot += dir * steps;
  Serial.print("c: "); Serial.println(currentRot);
  _delay_ms(100);
}

void step(void) {
  digitalWrite(STEP, HIGH);
  _delay_us(DRIVER_US_DATA_DELAY);
  digitalWrite(STEP, LOW);
  _delay_us(DRIVER_US_DATA_DELAY);
}

void setDirection(int8_t d) {
  if (d == 1) {
    digitalWrite(DIR, HIGH);
  }
  else if (d == -1) {
    digitalWrite(DIR, LOW);
  }
}

