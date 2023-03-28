//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <Pixy2.h>

Pixy2 pixy;

int duckWidth, duckHeight, redWidth, redHeight, greenWidth, greenHeight, whiteWidth, whiteHeight;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  // need to initialize pixy object
  pixy.init();
  // user color connected components program
  pixy.changeProg("color_connected_components");
}

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

void loop()
{
  static int16_t index = -1;
  Block *block=NULL;

  // DUCK
  Serial.println("Place Duck directly in front of pixy cam, 6 inches away");
  for(int i=10; i>0; i--) Serial.println("i");
  while(!block || block->m_age < 100)
  {
    pixy.ccc.getBlocks();
    if (index==-1) // search....
    {
      //Serial.println("Searching for block...");
      index = acquireBlock();
      if (index>=0)
        //Serial.println("Found block!");
    }
    // If we've found a block, find it, track it
    if (index>=0)
      block = trackBlock(index);

    // If we're able to track it, move motors
    if (block)
    {
      // print the block we're tracking -- wait until end of loop to reduce latency
      //block->print();
      Serial.print(".");
    }
  }
  duckWidth = block->m_width;
  duckHeight = block->m_height;
  Serial.println("");
  Serial.print("Duck Height: ");
  Serial.println(duckWidth);
  Serial.print("Duck Width: ");
  Serial.println(duckHeight);

  // RED
  Serial.println("");
  Serial.println("Place Red Pedestal directly in front of pixy cam, 6 inches away");
  for(int i=10; i>0; i--) Serial.println("i");
  while(!block || block->m_age < 100)
  {
    pixy.ccc.getBlocks();
    if (index==-1) // search....
    {
      //Serial.println("Searching for block...");
      index = acquireBlock();
      if (index>=0)
        //Serial.println("Found block!");
    }
    // If we've found a block, find it, track it
    if (index>=0)
      block = trackBlock(index);

    // If we're able to track it, move motors
    if (block)
    {
      // print the block we're tracking -- wait until end of loop to reduce latency
      //block->print();
      Serial.print(".");
    }
  }
  redWidth = block->m_width;
  redHeight = block->m_height;
  Serial.println("");
  Serial.print("Red Height: ");
  Serial.println(redWidth);
  Serial.print("Red Width: ");
  Serial.println(redHeight);

  // GREEN
  Serial.println("");
  Serial.println("Place Green Pedestal directly in front of pixy cam, 6 inches away");
  for(int i=10; i>0; i--) Serial.println("i");
  while(!block || block->m_age < 100)
  {
    pixy.ccc.getBlocks();
    if (index==-1) // search....
    {
      //Serial.println("Searching for block...");
      index = acquireBlock();
      if (index>=0)
        //Serial.println("Found block!");
    }
    // If we've found a block, find it, track it
    if (index>=0)
      block = trackBlock(index);

    // If we're able to track it, move motors
    if (block)
    {
      // print the block we're tracking -- wait until end of loop to reduce latency
      //block->print();
      Serial.print(".");
    }
  }
  greenWidth = block->m_width;
  greenHeight = block->m_height;
  Serial.println("");
  Serial.print("Green Height: ");
  Serial.println(greenWidth);
  Serial.print("Green Width: ");
  Serial.println(greenHeight);

  // WHITE
  Serial.println("");
  Serial.println("Place White Pedestal directly in front of pixy cam, 6 inches away");
  for(int i=10; i>0; i--) Serial.println("i");
  while(!block || block->m_age < 100)
  {
    pixy.ccc.getBlocks();
    if (index==-1) // search....
    {
      //Serial.println("Searching for block...");
      index = acquireBlock();
      if (index>=0)
        //Serial.println("Found block!");
    }
    // If we've found a block, find it, track it
    if (index>=0)
      block = trackBlock(index);

    // If we're able to track it, move motors
    if (block)
    {
      // print the block we're tracking -- wait until end of loop to reduce latency
      //block->print();
      Serial.print(".");
    }
  }
  whiteWidth = block->m_width;
  whiteHeight = block->m_height;
  Serial.println("");
  Serial.print("White Height: ");
  Serial.println(whiteWidth);
  Serial.print("White Width: ");
  Serial.println(whiteHeight);

  Serial.println("");
  Serial.println("DONE");
}

/* Block Tracking
void loop()
{  
  static int16_t index = -1;
  Block *block=NULL;
  
  pixy.ccc.getBlocks();

  if (index==-1) // search....
  {
    Serial.println("Searching for block...");
    index = acquireBlock();
    if (index>=0)
      Serial.println("Found block!");
 }
  // If we've found a block, find it, track it
  if (index>=0)
     block = trackBlock(index);

  // If we're able to track it, move motors
  if (block)
  {
    // print the block we're tracking -- wait until end of loop to reduce latency
    block->print();
  }  
  else // no object detected, stop motors, go into search state
  {
	  Serial.println("No objects detected");
    index = -1; // set search state
  }
}
*/

