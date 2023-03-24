#include "motors.h"
#include "pixy.h"
#include "nav.h"

driveMotors drive = driveMotors(1,2,3,4,5,6,7,8);

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  
  drive.forward(7);
}
