#include "motors.h"

driveMotors drive = driveMotors(1,2,3,4,5,6,7,8);

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:
  drive.forward(7);
}
