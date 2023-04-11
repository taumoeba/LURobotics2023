/* Motor control library for 2023 Lipscomb Robotics
*/

#include <Servo.h>
#include <Pixy2.h>
#include "Adafruit_VL53L0X.h"
#include "consts.h"
#include "pins.h"
// driveMotors isn't called by MainRobotControl directly
// it's used by navigation, which is called by MainRobotControl
class DriveMotors
{
    public:
    DriveMotors(int PWM1, int DIR1, int PWM2, int DIR2, int PWM3, int DIR3, int PWM4, int DIR4);
    // speed is 0-255
    void forward(int speed);
    // speed is 0-255
    void backward(int speed);
    // speed is 0-255
    void right(int speed);
    // speed is 0-255
    void left(int speed);
    // dir=0 is left, dir=1 is right. speed is 0-255
    void turn(int dir, int speed);
    void stop();

    private:
    int pwm1, dir1, pwm2, dir2, pwm3, dir3, pwm4, dir4;
};

class Arm
{
	public:
	Arm(int pwm, int dir, int step, int ms1, int ms2, int en, int slp);
	// 0=grabbing, 1=ducks, 2=turntable, 3=vertical
	void raise(int pos);
	// 0=grabbing, 1=ducks, 2=turntable
	void rotate(int pos);
	void sleep();
	void wake();

	private:
	int _dir, _step, _ms1, _ms2, _en, _slp, currRot, currRaise;
	// 0=grabbing, 1=ducks, 2=turntable
	const int[] rotateDegrees = {50, 200, 300}; // UPDATE WITH REAL 
	// 0=grabbing, 1=ducks, 2=turntable, 3=vertical
	const int[] raiseDegrees = {50, 200, 300, 360}; // UPDATE WITH REAL VALUES
	Servo raiseServo;
};

class Claw
{
	public:
	Claw(int PWM);
	void openClaw();
	void closeClaw();

	private:
	Servo clawServo;
	const int closedDegrees = 0; // subject to change
	const int openDegrees = 180; // subject to change
};

class Turntable
{
	public:
	Turntable(int doorDir, int doorPWM, int dir, int step, int ms1, int ms2, int en, int slp);
	// Stacks are 0, 1, and 2. 0.5, 1.5, and 2.5 are halfway between each stack.
	void goToOptimalStack(int color);
	float findFullStack();
	void emptyFullStack(float stack);
	void goToStack(float dest);
	void openDoor();
	void closeDoor();

	private:
	int _doorDir, _doorPWM, _dir, _step, _ms1, _ms2, _en, _slp;
	const int stepChunk = (60/360)*200;
	// stack over hole
	float currentStack = 0.0;
	// 0=empty, 1=red, 2=green, 3=white, 4=yellow
	int[3][3] stacks = {{0,0,0},{0,0,0},{0,0,0}};
	void sleep();
	void wake();
};

class DuckStorage
{
	public:
	DuckStorage(int solenoid, int pwm1, int pwm2);
	void tilt();
	void release();

	private:
	Servo servo1;
	Servo servo2;
	int _solenoid, _pwm1, _pwm2;
	const int tiltDegrees = 30;
};

class Navigation
{
	public:
	Navigation();
	void rotateToHeading(int h);
	void moveToWaypoint(int w);

	private:
	// middle of robot, so left edge is pos[0]-152
	int pos[2] = {1220,1032}; // starting position
	int targetPos[2];
	int recyclingPos[2];
	int maxPos[2] = {};
	// x,y,heading (0 is up, 90 is right, pixy is forward)
	const int waypoints[5][3] = {{1220,1032,0}, {200,1032,0}, {350,1032,0}, {,}};
	int lastWaypoint = 0;
	// Degrees. 0 is "up", 90 is right
	int heading = 0;

	DriveMotors drive = DriveMotors(DC_PWM1, DC_DIR1, DC_PWM2, DC_DIR2, DC_PWM3, DC_DIR3, DC_PWM4, DC_DIR4);
	Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
	Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
	Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
	Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
	VL53L0X_RangingMeasurementData_t measure1;
	VL53L0X_RangingMeasurementData_t measure2;
	VL53L0X_RangingMeasurementData_t measure3;
	VL53L0X_RangingMeasurementData_t measure4;

	void readDistance();
	void updatePos(int x, int y);
};

class SmartPixy
{
	public:
	SmartPixy();
	// Take the biggest block (blocks[0]) that's been around for at least 30 frames (1/2 second)
	// and return its index, otherwise return -1
	int16_t acquireBlock();
	// Find the block with the given index.  In other words, find the same object in the current
	// frame -- not the biggest object, but he object we've locked onto in acquireBlock()
	// If it's not in the current frame, return NULL
	Block *trackBlock(uint8_t index);

	// Set x to choose returned value.
	// 0=signature, 1=x_pos, 2=y_pos, 3=width, 4=height
	int returnBlockInfo(int x);

	private:
	Pixy2 pixy;
};
