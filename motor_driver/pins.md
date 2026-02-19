

#include <TCA9548A.h>
#include <AS5600.h>

//stepper pins
const byte dirPinS0 = 5;
const byte stepPinS0 = 17;


const byte dirPinS1 = 18;
const byte stepPinS1 = 19;

const byte dirPinS2 = 32;
const byte stepPinS2 = 33;

const byte dirPinS3 = 25;
const byte stepPinS3 = 26;


const int stepsPerRev = 12835;
const int stepsPerDegree = stepsPerRev / 360;
const int maxDegrees = 135;

//i2c multiplexer
/ Define AS5600 objects
AS5600 stpA;
AS5600 stpB;
AS5600 stpC;
AS5600 stpD;

// I2C ports
const int I2CA = 2;
const int I2CB = 0;
const int I2CC = 1;
const int I2CD = 3;

// Initialize TCA9548 and AS5600 sensors
  TCA9548(I2CB);
  stpB.begin(4);
  stpB.setDirection(AS5600_CLOCK_WISE);
  
  TCA9548(I2CD);
  stpD.begin(4);
  stpD.setDirection(AS5600_CLOCK_WISE);
  
  TCA9548(I2CA);
  stpA.begin(4);
  stpA.setDirection(AS5600_CLOCK_WISE);
  
  TCA9548(I2CC);
  stpC.begin(4);
  stpC.setDirection(AS5600_CLOCK_WISE);
  
  
  //in loop
  
   // Read current positions from AS5600 sensors
    TCA9548(I2CB);
    currentPos0 = (stpB.rawAngle() * AS5600_RAW_TO_DEGREES)- (cs0);

    TCA9548(I2CA);
    currentPos1 = (stpA.rawAngle() * AS5600_RAW_TO_DEGREES)- (cs1);
    //currentPos1 = currentPos1 *-1;

    TCA9548(I2CD);
    currentPos2 = (stpD.rawAngle() * AS5600_RAW_TO_DEGREES)- (cs2);

    TCA9548(I2CC);
    currentPos3 = (stpC.rawAngle() * AS5600_RAW_TO_DEGREES)- (cs3);

