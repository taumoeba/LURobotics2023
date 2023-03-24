#include "motors.h"
#include "nav.h"
#include "pins.h"
#include <Pixy2.h>

driveMotors drive = driveMotors(DC_PWM1, DC_DIR1, DC_PWM2, DC_DIR2, DC_PWM3, DC_DIR3, DC_PWM4, DC_DIR4);
arm arm = arm(PWM6, STEP_DIR1, STEP1, MS1_1, MS2_1, EN1, SLP1);
claw claw = claw(PWM7);
turntable turntable = turntable(DC_DIR5, DC_PWM5, STEP_DIR2, STEP2, MS1_2, MS2_2, EN2, SLP2);
duckStorage ducks = duckStorage(SOL, PWM8, PWM9);
Pixy2 pixy;

// Take the biggest block (blocks[0]) that's been around for at least 30 frames (1/2 second)
// and return its index, otherwise return -1
int16_t acquireBlock()
{
  if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age>30)
    return pixy.ccc.blocks[0].m_index;

  return -1;
}

// Find the block with the given index.  In other words, find the same object in the current
// frame -- not the biggest object, but he object we've locked onto in acquireBlock()
// If it's not in the current frame, return NULL
Block *trackBlock(uint8_t index)
{
  uint8_t i;

  for (i=0; i<pixy.ccc.numBlocks; i++)
  {
    if (index==pixy.ccc.blocks[i].m_index)
      return &pixy.ccc.blocks[i];
  }

  return NULL;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  pixy.init();
  pixy.changeProg("color_connected_components");
}

void loop() {
  // PIXY CAM STUFF -----------------------------------------------------
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

  // If we're able to track it, print stuff
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
  // END PIXY CAM STUFF --------------------------------------------------

  //drive.forward(7);
}
