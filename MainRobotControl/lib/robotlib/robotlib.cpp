/* Motor control library for 2023 Lipscomb Robotics
*/


#ifndef ARDUINO_H
#define ARDUINO_H

#ifndef PIXY2_H
#define PIXY2_H

#include "robotlib.h"

/*******************************
 * DriveMotors
 * supposedly done
********************************/
// DriveMotors isn't called by MainRobotControl directly
// it's used by Navigation, which is called by MainRobotControl
DriveMotors::DriveMotors(int PWM1, int DIR1, int PWM2, int DIR2, int PWM3, int DIR3, int PWM4, int DIR4)
{
	if(DEBUG) Serial.println("Initializing drive motors");
	pwm1 = PWM1;
	pwm2 = PWM2;
	pwm3 = PWM3;
	pwm4 = PWM4;
	dir1 = DIR1;
	dir2 = DIR2;
	dir3 = DIR3;
	dir4 = DIR4;

	pinMode(pwm1, OUTPUT);
	pinMode(pwm2, OUTPUT);
	pinMode(pwm3, OUTPUT);
	pinMode(pwm4, OUTPUT);
	pinMode(dir1, OUTPUT);
	pinMode(dir2, OUTPUT);
	pinMode(dir3, OUTPUT);
	pinMode(dir4, OUTPUT);

	digitalWrite(dir1, HIGH);
	digitalWrite(dir2, HIGH);
	digitalWrite(dir3, HIGH);
	digitalWrite(dir4, LOW);
}

void DriveMotors::forward(int speed)
{
	if(DEBUG) Serial.println("Driving forward");
	digitalWrite(dir1, HIGH);
	digitalWrite(dir3, HIGH);
	
	if(speed > 255) speed = 255;
	if(speed < 0) speed = 0;

	analogWrite(pwm1, speed);
	analogWrite(pwm3, speed);
}

void DriveMotors::backward(int speed)
{
	if(DEBUG) Serial.println("Driving backward");
	digitalWrite(dir1, LOW);
	digitalWrite(dir3, LOW);
	
	if(speed > 255) speed = 255;
	if(speed < 0) speed = 0;

	analogWrite(pwm1, speed);
	analogWrite(pwm3, speed);
}

void DriveMotors::right(int speed)
{
	if(DEBUG) Serial.println("Driving right");
	digitalWrite(dir2, LOW); // test
	digitalWrite(dir4, HIGH);
	
	if(speed > 255) speed = 255;
	if(speed < 0) speed = 0;

	analogWrite(pwm2, speed);
	analogWrite(pwm4, speed);
}

void DriveMotors::left(int speed)
{
	if(DEBUG) Serial.println("Driving left");
	digitalWrite(dir2, HIGH); // test
	digitalWrite(dir4, LOW);
	
	if(speed > 255) speed = 255;
	if(speed < 0) speed = 0;

	analogWrite(pwm2, speed);
	analogWrite(pwm4, speed);
}

void DriveMotors::turn(bool dir, int speed)
{
	if(DEBUG) Serial.println("Turning");
	// experiment with directions but i think they should all be the same
	digitalWrite(dir1, dir);
	digitalWrite(dir2, !dir);
	digitalWrite(dir3, dir);
	digitalWrite(dir4, dir);
	analogWrite(pwm1, speed);
	analogWrite(pwm2, speed);
	analogWrite(pwm3, speed);
	analogWrite(pwm4, speed);
}

void DriveMotors::stop()
{
	if(DEBUG) Serial.println("Stopping");
	analogWrite(pwm1,0);
	analogWrite(pwm2,0);
	analogWrite(pwm3,0);
	analogWrite(pwm4,0);
}


/************************************
 * Arm
 * maybe done
*************************************/
Arm::Arm(int pwm, int dir, int step, int ms1, int ms2, int en, int slp)
{
	if(DEBUG) Serial.println("Initializing arm");
	pinMode(_dir, OUTPUT);
	pinMode(_step, OUTPUT);
	pinMode(_ms1, OUTPUT);
	pinMode(_ms2, OUTPUT);
	pinMode(_en, OUTPUT);
	pinMode(_slp, OUTPUT);
	pinMode(pwm, OUTPUT);

	//Servo raiseServo;
	raiseServo.attach(pwm);
	_dir = dir;
	_step = step;
	_ms1 = ms1;
	_ms2 = ms2;
	_en = en;
	_slp = slp;

	// en disables outputs when high
	// slp minimizes power consumption when low
	// rst resets internal translator when low
	digitalWrite(_dir, LOW);
	digitalWrite(_en, LOW);
	digitalWrite(_slp, LOW);
	digitalWrite(_step, LOW);
	// starts up in sleep mode

	currRot = 0; // REPLACE WITH STARTING ROTATION DEGREES
	currRaise = 0; // REPLACE WITH STARTING RAISE DEGREES
}

void Arm::raise(int pos)
{
	if(DEBUG) Serial.println("Raising arm");
	if(raiseDegrees[pos] > currRaise)
	{
		for(int i=currRaise; i<raiseDegrees[pos]; i++)
		{
			raiseServo.write(i);
			delay(0.1); // TEST DELAY
		}
	} else if(raiseDegrees[pos] < currRaise)
	{
		for(int i=currRaise; i>raiseDegrees[pos]; i--)
		{
			raiseServo.write(i);
			delay(0.1); // TEST DELAY
		}
	}
	currRaise = raiseDegrees[pos];
}

// negative degrees to go counterclockwise
void Arm::rotate(int pos)
{
	if(DEBUG) Serial.println("Rotating arm");
	// optional microstepping
	// digitalWrite(MS1, HIGH); //Pull MS1, and MS2 high to set logic to 1/8th microstep resolution
	// digitalWrite(MS2, HIGH);
	if(rotateDegrees[pos]<currRot) digitalWrite(_dir, HIGH); // TEST DIRECTIONS
	else digitalWrite(_dir, LOW);

	int steps = (abs(rotateDegrees[pos])/360)*200;
	for(int i=0; i<steps; i++)
	{
		digitalWrite(_step, HIGH);
		delay(0.1); // TEST DELAY
		digitalWrite(_step, LOW);
		delay(0.1);
	}
	currRot = rotateDegrees[pos];
}

void Arm::sleep()
{
	if(DEBUG) Serial.println("Putting arm to sleep");
	digitalWrite(_slp, LOW);
}

void Arm::wake()
{
	if(DEBUG) Serial.println("Waking up arm");
	digitalWrite(_slp, HIGH);
}


/*********************************
 * Claw
 * supposedly done
**********************************/
Claw::Claw(int PWM)
{
	if(DEBUG) Serial.println("Initializing claw");
	pinMode(PWM, OUTPUT);
	clawServo.attach(PWM);
	//Servo myservo;
}

void Claw::openClaw()
{
	if(DEBUG) Serial.println("Opening claw");
	for(int i=0; i<openDegrees; i++)
	{
		clawServo.write(i);
		delay(0.1);
	}
}

void Claw::closeClaw()
{
	if(DEBUG) Serial.println("Closing claw");
	for(int i=openDegrees; i>closedDegrees; i--)
	{
		clawServo.write(i);
		delay(0.1);
	}
}


/*************************************
 * Turntable
 * maybe done
***************************************/
Turntable::Turntable(int doorDir, int doorPWM, int dir, int step, int ms1, int ms2, int en, int slp)
{
	if(DEBUG) Serial.println("Initializing turntable");
	_doorDir = doorDir;
	_doorPWM = doorPWM;
	_dir = dir;
	_step = step;
	_ms1 = ms1;
	_ms2 = ms2;
	_en = en;
	_slp = slp;

	pinMode(_doorDir, OUTPUT);
	pinMode(_doorPWM, OUTPUT);
	pinMode(_dir, OUTPUT);
	pinMode(_step, OUTPUT);
	pinMode(_ms1, OUTPUT);
	pinMode(_ms2, OUTPUT);
	pinMode(_en, OUTPUT);
	pinMode(_slp, OUTPUT);

	digitalWrite(_doorDir, HIGH);

	// en disables outputs when high
	// slp minimizes power consumption when low
	// rst resets internal translator when low
	digitalWrite(_dir, HIGH);
	digitalWrite(_en, LOW);
	digitalWrite(_slp, LOW);
	// starts up in sleep mode
}

// Go to stack best suited for containing currently-held pedestal
// Stacks are 0, 1, and 2. 0.5, 1.5, and 2.5 are halfway between each stack.
// correct pedestal order (bottom to top): white, green, red
int Turntable::goToOptimalStack(int color)
{
	if(DEBUG) Serial.println("Rotating to optimal stack");
	// deciding stack
	// colors: 1=red, 2=green, 3=white
	float dest = 10.0; // 10 means stack not found
	if(color==3) // white
	{
		for(int i=0; i<3; i++)
		{
			if(stacks[i][0]==0) // bottom is empty
			{
				dest=i;
				stacks[i][0]=color;
				break;
			}
		}
	} else if(color==2) // green
	{
		for(int i=0; i<3; i++)
		{
			if(stacks[i][0]==3 && stacks[i][1]==0) // bottom is white & middle is empty
			{
				dest=i;
				stacks[i][1]=color;
				break;
			}
		}
	} else if(color==1) // red
	{
		for(int i=0; i<3; i++)
		{
			if(stacks[i][1]==2 && stacks[i][2]==0) // middle is green & top is empty
			{
				dest=i;
				stacks[i][2]=color;
				break;
			}
		}
	}

	if(dest==10.0) // optimal stack was not found
	{
		// just look for a stack with an empty spot
		for(int i=0; i<3; i++)
		{
			for(int j=0; j<3; j++)
			{
				if(stacks[i][j]==0) // if empty spot exists
				{
					dest=i;
					stacks[i][j]=color;
					break;
				}
			}
		}
		if(dest==10.0) return -1; // no empty spots exist, return error code
	}

	goToStack(dest);
	return 0;
}

void Turntable::goToStack(float dest)
{
	if(DEBUG) Serial.println("Going to stack");
	// moving to stack
	// TEST
	if(currentStack-dest>0) 
	{
		digitalWrite(_dir, HIGH); // go clockwise
		while(currentStack!=dest)
		{
			for(int i=0; i<stepChunk; i++)
			{
				digitalWrite(_step, HIGH);
				delay(0.1);
				digitalWrite(_step, LOW);
				delay(0.1);
			}
			currentStack -= 0.5;
		}
	}
	else 
	{
		digitalWrite(_dir, LOW); // go counter-clockwise
		while(currentStack!=dest)
		{
			for(int i=0; i<stepChunk; i++)
			{
				digitalWrite(_step, HIGH);
				delay(0.1);
				digitalWrite(_step, LOW);
				delay(0.1);
			}
			currentStack += 0.5;
		}
	}
}

float Turntable::findFullStack()
{
	if(DEBUG) Serial.println("Finding full stack");
	float stack = -1.0;
	for(int i=0; i<2; i++)
	{
		if(stacks[i][2]!=0) // top spot of stack is full
		{
			stack = i;
		}
	}
	return stack;
}

void Turntable::emptyFullStack(float stack)
{
	if(DEBUG) Serial.println("Emptying full stack");
	goToStack(stack);
	openDoor();
	delay(1000); // TEST
	closeDoor();
	stacks[(int)stack][0]=0;
	stacks[(int)stack][1]=0;
	stacks[(int)stack][2]=0;
}

void Turntable::openDoor()
{
	if(DEBUG) Serial.println("Opening door");
	digitalWrite(_doorDir, HIGH); // test direction
	analogWrite(_doorPWM, 127); // test speed
	delay(1000); // test duration
	analogWrite(_doorPWM, 0);
}

void Turntable::closeDoor()
{
	if(DEBUG) Serial.println("Closing door");
	digitalWrite(_doorDir, LOW); // test direction
	analogWrite(_doorPWM, 127); // test speed
	delay(1000); // test duration
	analogWrite(_doorPWM, 0);
}

void Turntable::sleep()
{
	if(DEBUG) Serial.println("Putting turntable to sleep");
	digitalWrite(_slp, LOW);
}

void Turntable::wake()
{
	if(DEBUG) Serial.println("Waking turntable up");
	digitalWrite(_slp, HIGH);
}


/*****************************************
 * DuckStorage
 * supposedly done
******************************************/
DuckStorage::DuckStorage(int solenoid, int pwm1, int pwm2)
{
	if(DEBUG) Serial.println("Initializing duck storage");
	_solenoid = solenoid;
	_pwm1 = pwm1;
	_pwm2 = pwm2;

	pinMode(_solenoid, OUTPUT);
	pinMode(_pwm1, OUTPUT);
	pinMode(_pwm2, OUTPUT);

	servo1.attach(pwm1);
	servo2.attach(pwm2);
	servo1.write(0);
	servo2.write(0);
}

void DuckStorage::tilt()
{
	if(DEBUG) Serial.println("Tilting duck storage");
	for(int i=1; i<=tiltDegrees; i++)
	{
		servo1.write(i);
		servo2.write(i);
		delay(100); // adjust as needed
	}
}

void DuckStorage::release()
{
	if(DEBUG) Serial.println("releasing duck storage");
	digitalWrite(_solenoid, HIGH);
	// we could probably just get rid of these two lines if we're short on time
	delay(2000);
	digitalWrite(_solenoid, LOW);
}


/***********************************
 * Navigation
 * maybe done
************************************/
Navigation::Navigation()
{
	if(DEBUG) Serial.println("Initializing navigation");
	// Pin setup
	pinMode(XSHUT1, OUTPUT);
	pinMode(XSHUT2, OUTPUT);
	pinMode(XSHUT3, OUTPUT);
	pinMode(XSHUT4, OUTPUT);

	// Distance sensor setup
	// reset sensors
	digitalWrite(XSHUT1, LOW);
	digitalWrite(XSHUT2, LOW);
	digitalWrite(XSHUT3, LOW);
	digitalWrite(XSHUT4, LOW);
	delay(10);
	digitalWrite(XSHUT1, HIGH);
	digitalWrite(XSHUT2, HIGH);
	digitalWrite(XSHUT3, HIGH);
	digitalWrite(XSHUT4, HIGH);

	// set sensor addresses one-by-one
	digitalWrite(XSHUT2, LOW);
	digitalWrite(XSHUT3, LOW);
	digitalWrite(XSHUT4, LOW);
	lox1.begin(0x30);
	digitalWrite(XSHUT2, HIGH);
	lox2.begin(0x31);
	digitalWrite(XSHUT3, HIGH);
	lox3.begin(0x32);
	digitalWrite(XSHUT4, HIGH);
	lox4.begin(0x33);

	currPos[0] = waypoints[0][0];
	currPos[1] = waypoints[0][1];
}

void Navigation::moveToWaypoint(int w)
{
	if(DEBUG) Serial.println("Moving to waypoint");
	// might work
	// larger target x means drive right
	// larger target y means drive backward (down)
	targetPos[0] = waypoints[w][0];
	targetPos[1] = waypoints[w][1];

	int horiDiff = targetPos[0] - currPos[0];
	int vertDiff = targetPos[1] - currPos[1];

	if(horiDiff>0)
	{
		drive.right(driveSpeed);
		delay(milliPerMilli*abs(horiDiff));
		drive.stop();
	} else if(horiDiff<0)
	{
		drive.left(driveSpeed);
		delay(milliPerMilli*abs(horiDiff));
		drive.stop();
	}
	if(vertDiff>0)
	{
		drive.backward(driveSpeed);
		delay(milliPerMilli*abs(horiDiff));
		drive.stop();
	} else if(vertDiff<0)
	{
		drive.forward(driveSpeed);
		delay(milliPerMilli*abs(horiDiff));
		drive.stop();
	}
}
/*
void Navigation::rotateToHeading(int h)
{
	// check current heading
	// rotate if necessary
}
*/
void Navigation::readDistance()
{
	if(DEBUG) Serial.println("reading distance sensors");
	lox1.rangingTest(&measure1, false); 
	lox2.rangingTest(&measure2, false);
	lox3.rangingTest(&measure3, false);
	lox4.rangingTest(&measure4, false);
}

/***************************************
 * SmartPixy
 * maybe done
****************************************/
SmartPixy::SmartPixy()
{
	if(DEBUG) Serial.println("Initializing pixy camera");
	//Pixy2 pixy;
	pixy.init();
	pixy.changeProg("color_connected_components");
	pixy.setLamp(1,1);
}

// Take the biggest block (blocks[0]) that's been around for at least 30 frames (1/2 second) and return its index, otherwise return -1
int16_t SmartPixy::acquireBlock()
{
	if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age>30)
		return pixy.ccc.blocks[0].m_index;

	return -1;
}

// Find the block with the given index.  In other words, find the same object in the current
// frame -- not the biggest object, but he object we've locked onto in acquireBlock()
// If it's not in the current frame, return NULL
Block* SmartPixy::trackBlock(uint8_t index)
{
	uint8_t i;

	for (i=0; i<pixy.ccc.numBlocks; i++)
	{
		if (index==pixy.ccc.blocks[i].m_index)
		return &pixy.ccc.blocks[i];
	}

	return NULL;
}

int SmartPixy::returnBlockInfo(int x)
{
	if(DEBUG) Serial.println("returning block info");
	static int16_t index = -1;
	Block *block=NULL;

	for(int i=10; i>0; i--) Serial.println("i");
	while(!block || block->m_age < 100)
	{
		pixy.ccc.getBlocks();
		if (index==-1) // search....
		{
			Serial.println("Searching for block...");
			index = acquireBlock();
			if (index>=0)
				Serial.println("Found block!");
		}
		// If we've found a block, find it, track it
		if (index>=0) block = trackBlock(index);

		// If we're able to track it, report info
		if (block) 
		{
			// colors: 1=red, 2=green, 3=white, 4=yellow
			if(x==0) return block->m_signature;
			else if(x==1) return block->m_x;
			else if(x==2) return block->m_y;
			else if(x==3) return block->m_width;
			else if(x==4) return block->m_height;
			else return -1;
		}
	}
	return -1;
}

#endif
#endif