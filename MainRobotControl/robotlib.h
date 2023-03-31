/* Motor control library for 2023 Lipscomb Robotics
*/

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
  void raise(int degrees);
  // negative degrees to go counterclockwise
  void rotate(int degrees);
  void sleep();
  void wake();

  private:
  int _dir, _step, _ms1, _ms2, _en, _slp;
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
  void goToStackPos(float dest);
  void openDoor();
  void closeDoor();
  void sleep();
  void wake();

  private:
  int _doorDir, _doorPWM, _dir, _step, _ms1, _ms2, _en, _slp;
  const int stepChunk = (60/360)*200;
  // stack over hole
  float currentStack = 0.0;
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
  void updatePos(int x, int y);
  void moveToWaypoint(int w);

  private:
  int pos[2];
  int targetPos[2];
  int recyclingPos[2];
  int maxPos[2] = {};
  const int waypoints[5][2] = {{0,0}, {0,0}, {0,0}, {0,0}, {0,0}}; // update
  int lastWaypoint = 0;
  // Degrees. 0 is "up", 90 is right
  int heading = 0;
  driveMotors drive = driveMotors(DC_PWM1, DC_DIR1, DC_PWM2, DC_DIR2, DC_PWM3, DC_DIR3, DC_PWM4, DC_DIR4);
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

  /* PIXY CAM STUFF -----------------------------------------------------
  static int16_t blockIndex = -1;
  Block *block=NULL;
  pixy.ccc.getBlocks();
  if (blockIndex==-1) // search....
  {
    Serial.println("Searching for block...");
    blockIndex = acquireBlock();
    if (blockIndex>=0)
      Serial.println("Found block!");
  }
  // If we've found a block, find it, track it
  if (blockIndex>=0)
     block = trackBlock(blockIndex);

  // If we're able to track it, do stuff
  if (block)
  {
	  Serial.println("Tracking block!");
    block->print();
  }
  else // no object detected, go into search state
  {
	  Serial.println("No objects detected");
    index = -1; // set search state
  }
  // END PIXY CAM STUFF --------------------------------------------------*/

  private:
  Pixy2 pixy;
};

#endif
#endif
#endif