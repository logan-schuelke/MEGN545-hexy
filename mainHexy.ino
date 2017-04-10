/*
Main code for Hexy Robot
*/

// BasicLinearAlgebra - Version: Latest 
#include <BasicLinearAlgebra.h>
#include <MemoryDelegate.hpp>

#include <ros.h>
#include <SPI.h>
#include <Pixy.h>
#include <math.h>

Pixy pixy;
int j;
int x;
int y;
uint16_t blocks;
char buf[32]; 
const int target = 2; // any number from 1-7 for normal signatures, orange is taught on signature 2
//float error = 0.3;
//float dist;
const int maxSize = 32000L; // closest that Hexy can get to Pixy, max camera pixel size is 64000 (320*200)
const int xCenter = 160L; // center of Pixy lens
const int yCenter = 100L;

void setup() {
    Serial.begin(9600);
    Serial.print("Starting...\n");
    
    pixy.init();
}

void loop() {
  // Hexy looks for blocks, rotates to center largest target block in Pixy view, 
  // takes one step towards or away from it, then repeats looking for blocks
  // resource: http://www.cmucam.org/projects/cmucam5/wiki/LEGO_Chase_Demo
  // http://www.cmucam.org/projects/cmucam5/wiki/PID_LEGO_Block
  

  // look for blocks
  blocks = pixy.getBlocks(); 
  // this function returns number of blocks (objects) detected, starting from 1
  
  // If the block we want is in view, rotate Hexy to face it, then take 1 step towards or away from it
  if (blocks) { // if blocks are detected...
    int size = 0;
    int blockNum;
    for (j=0; j<blocks; j++) {
      if (pixy.blocks[j].signature == target) {
        //blockLocation(j); // calculates x, y, dist of object wrt Hexy coordinates
        
        // if multiple blocks detected in target signature, choose largest object to follow
        w = pixy.blocks[j].width;
        h = pixy.blocks[j].height;
        int jsize = w*h; 
        if (jsize >= size) {
            size = jsize;
            blockNum = j;
        }
      }
    }
    x = pixy.blocks[blockNum].x;
    y = pixy.blocks[blockNum].y;
    int xError = xCenter - x;
    int yError = yCenter - y;
    if (xError > 0) {
        //rotateCCW(xError); // rotate by xError to position object in center of Pixy's view
        Serial.print("Hexy rotate CCW\n");
    } 
    else if (xError < 0) {
      //rotateCW(xError);
      Serial.print("Hexy rotate CW\n");
    }
    if (size <= maxSize) {
      //walkForward(1); // 1 step
      Serial.print("Hexy walk FORWARD\n");
    }
    else if (size >= maxSize){
      //walkBackward(1);
      Serial.print("Hexy walk BACKWARD\n");
    }
  } 
}

// GAIT FUNCTIONS

// COMPUTER VISION FUNCTIONS // currently not using any of the below

/* blockLocation() finds the x and y coordinates of the block that we want Pixy to identify
  it also scales the object to find the distance Hexy is away from object
  If the object is always a certain size, scaling it 
  allows the robot to calculate distance.
*/
void blockLocation(int j)
{
  // j is the location in the block array that matches the target
  x = pixy.blocks[j].x;
  y = pixy.blocks[j].y;
  w = pixy.blocks[j].width;
  h = pixy.blocks[j].height; 
  
  // Compare width and height to some nominal width/height and scale
  // Let's say nominal is 1 ft away from Hexy, and we use a 2 in diameter ball
  // Pixy may read this width and height as 100 x 100
  // Need to find this relationship between distance and pixels
  wscale = w/100; // so if w is 100, wcale is 1, object is 1 ft from Hexy
  hscale = h/100;
  dist = (wscale + hscale)/2; // in feet
  
  // Transform the x and y coordinates of the block to the Hexy coordinates
  x = pixy2hexy(x);
  y = pixy2hexy(y);
}

// pixy2hexy() transforms the Pixy object coordinates to Hexy coordinates
int pixy2hexy(int pixycoord[], int n)
{
  // pixycoord[] is an 1x2 array with x and y coordinates
  // n is the size of the array, should be 2
   float transform[4][4] = {
    { cos(30), -sin(30), 0, 5 },
    { sin(30), cos(30), 0, 4 },
    { 0, 0, 1, 0 },
    { 0, 0, 0, 1 }
  }; // calculate this transformation matrix - this is arbitrary, need to fix
  Matrix<2,1> P(pixycoord);
  Matrix<4,4> T(transform);
  Matrix<2,1> hexycoord;
  Multiply(P,T,hexycoord);
  return hexycoord;
}

/* This function simply prints the detected object blocks 
 (including color codes) through the serial console.  It uses the Arduino's 
 ICSP port. (see hello_world.ino example in Pixy folder)
 It prints the detected blocks once per second because printing all of the 
 blocks for all 50 frames per second would overwhelm the Arduino's serial port.
*/
void printBlocks()
{ 
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        pixy.blocks[j].print();
      }
    }
  }  
}

