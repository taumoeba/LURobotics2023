#include "robotlib.h"

// Object declaration
ControlState s = foodChipDropoff;
Navigation nav = Navigation();
Arm arm = Arm(PWM6, STEP_DIR1, STEP1, MS1_1, MS2_1, EN1, SLP1);
Claw claw = Claw(PWM7);
Turntable turntable = Turntable(DC_DIR5, DC_PWM5, STEP_DIR2, STEP2, MS1_2, MS2_2, EN2, SLP2);
DuckStorage ducks = DuckStorage(SOL, PWM8, PWM9);
SmartPixy pixy = SmartPixy();

// global variables
int currentWaypoint = 0;
int blockInfo[5]; // 0=signature, 1=x_pos, 2=y_pos, 3=width, 4=height
unsigned long startTime = millis();

void setup() 
{
	// Serial setup
	Serial.begin(115200);
	Serial.println("Starting...");

	// initial position reading
  /*
	readDistance();
	Serial.print("Sensor 1: ");
	Serial.println(measure1.RangeMilliMeter);
	Serial.print("Sensor 2: ");
	Serial.println(measure2.RangeMilliMeter);
	Serial.print("Sensor 3: ");
	Serial.println(measure3.RangeMilliMeter);
	Serial.print("Sensor 4: ");
	Serial.println(measure4.RangeMilliMeter);
  */

 
}



void loop() 
{
	if(millis()-startTime > 160000 || currentWaypoint==24) s = recycling; // go to recycling with 20 seconds left or when last waypoint reached (TEST)
	// execute operations according to control state
	if(s==foodChipDropoff)
	{
		// Go to square one
		currentWaypoint++;
		nav.moveToWaypoint(currentWaypoint);
		// Drop first batch of chips
		turntable.goToStack(1.0);
		turntable.openDoor();
		delay(1000); // TEST
		turntable.closeDoor();
		// Go to square two
		currentWaypoint++;
		nav.moveToWaypoint(currentWaypoint);
		// Drop second batch of chips
		turntable.goToStack(2.0);
		turntable.openDoor();
		delay(1000); // TEST
		turntable.closeDoor();
		// Switch to next state
		s = searching;
	}
	else if(s==searching)
	{
		float fullStack=turntable.findFullStack();
		if(fullStack!=-1.0) // full stack was found
		{
      // UPDATE TO AVOID DRIVING OVER STACK WHEN GOING RIGHT TO LEFT
			nav.moveToWaypoint(currentWaypoint-1); // drive to previous waypoint
			// rotate for optimal stack dropping
			turntable.emptyFullStack(fullStack);
			// rotate back
			nav.moveToWaypoint(currentWaypoint); // move back to current waypoint
		}
		currentWaypoint++;
		nav.moveToWaypoint(currentWaypoint);
		// colors: 1=red, 2=green, 3=white, 4=yellow
		//blockInfo = {pixy.returnBlockInfo(0), pixy.returnBlockInfo(1), pixy.returnBlockInfo(2), pixy.returnBlockInfo(3), pixy.returnBlockInfo(4)};
    blockInfo[0] = pixy.returnBlockInfo(0);
    blockInfo[0] = pixy.returnBlockInfo(1);
    blockInfo[0] = pixy.returnBlockInfo(2);
    blockInfo[0] = pixy.returnBlockInfo(3);
    blockInfo[0] = pixy.returnBlockInfo(4);
		s = intercepting;
	}
	else if(s==intercepting)
	{
		if(abs(blockInfo[2]-158)>(clawCenterWindow+armOffset))
		{
      // UPDATE/FINISH
			// move to center claw on object
		}
		s = grabbing;
	}
	else if(s==grabbing)
	{
		arm.raise(3); // make arm vertical before rotating to avoid collisions
		arm.rotate(0);
		arm.raise(0);
		claw.closeClaw();
		if(blockInfo[0]==4) // object is duck
		{
			arm.raise(3);
			arm.rotate(1);
			claw.openClaw();
		} else // object is pedestal
		{
			if(turntable.goToOptimalStack(blockInfo[0])!=-1) // spot found, put in turntable
			{
				arm.raise(3);
				arm.rotate(2);
				claw.openClaw();
				delay(500);
				claw.closeClaw();
			}
			else // turntable is full, throw in duck storage (should never happen)
			{
				arm.raise(3);
				arm.rotate(1);
				claw.openClaw();
				delay(500);
				claw.closeClaw();
			}
		}
		s = searching;
	}
	else if(s==recycling)
	{
		// recycling is waypoint 25
		nav.moveToWaypoint(25);
		//nav.rotateToHeading(180); // should already be in correct orientation
		ducks.tilt();
		ducks.release();
		while(true) delay(1000); // end game
	}
  else s=searching; // only happens if s gets set to some nonsense value
}