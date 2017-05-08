/*
  Main code for Hexy Robot

  Update 5/4/17 - Added hexapod_forwardMovement_outline functions and variables

  Update 5/7/17 - Added hexapod_test_turn functions and variables
                Changing to MEGA

  Update 5/8/17 - Contains changes to MEGA

*/

#include <Servo.h>
Servo servos[18];
#include <SPI.h>
#include <Pixy.h>
//#include <ros.h>

/////////////////// Pixy variables ///////////////////////
Pixy pixy;
int j;
int x;
int y;
int w;
int h;
uint16_t blocks;
char buf[32];
const int target = 2; // any number from 1-7 for normal signatures, orange is taught on signature 2
//float error = 0.3;
float dist;
const int maxSize = 3600L; // closest that Hexy can get to Pixy, max camera pixel size is 64000 (320*200)
const int xCenter = 160L; // center of Pixy lens
const int yCenter = 100L;
const int xBand = 80L; // allowable pixels block can be off-center in x
const int sBand = 4500L; // size range that Hexy will not move forward or backward
unsigned long lastEventTime = 0;

//////////////// Movement Variables/////////////////////
//Calibration offsets:
const float bodyAngle = 49.22;
// Enter calibration angle for each servo here:
const float calibOff[] = { -bodyAngle, 0, 0, 0, 0, 0, bodyAngle, 0, 0, -bodyAngle, 0, 0, 0, 0, 0, bodyAngle, 0, 0};
const float minAngle = 10;
const float maxMiddleAngle = (bodyAngle-minAngle)/2;
boolean tripodLifted[] = {false, false};
boolean fullStep[] = {false, false};

const int tripod1 = 0;
const int tripod2 = 1;

const short START_POS_HIP = (0);
const short START_POS_KNEE = (45);
const short START_POS_ANKLE = (45);

// Leg measurements in mm: Update - switched values for TIBIA and FEMUR
const float TIBIA = 49.0;
const float FEMUR = 52.0;
const float COXA = 26.0;
//Here's pi:
const float pi = 3.14159265359;

//Here are some contants that might need to be changed:
const float maxStep = 45.92; // this is max z distance foot can move forward
const float minStep = 0; // this is max z distance foot can move backward
const short stepIncrement = 10; // how far each foot moves every loop- lower = smoother but slower
const float yStand = TIBIA + FEMUR * sin(START_POS_KNEE*pi / 180); // height of body relative to foot while foot is planted.
const float yLift = yStand - 15; // height of body relative to foot while foot is lifted.
const float xF = 39.6128; // lateral distance between foot and body for front feet (also back), while striding
const float xM = 60.6482; // lateral distance between foot and body for middle feet, while striding
// All delays in ms:
const int dropDelay = 800; // How long to wait after telling legs to drop
const int readyDelay = 1000; // How to wait for legs to get into start positoin
const int incrementDelay = 0; // How long to wait each incremental movement

float footPos[] = {maxStep, maxStep}; // z position of front right foot, front left foot
float z[] = {0, 0}; // delta z of tripod 1 & 2

// Initialize joint angle vectors:
float servoAngles[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE
                      };

// To get array element number: (leg name)*LEG + (joint name)
//    ex: right front knee => RF*LEG + KNEE = 1*3 + -2 = 1
// To get pin numnber: (leg name)*LEG + (joint name) + pinOffset
const int RF = 4; const int RM = 5; const int RB = 6;
const int LF = 1; const int LM = 2; const int LB = 3;
const int LEG = 3; const int HIP = -3; const int KNEE = -2; const int ANKLE = -1;

// pinOffset allows servos to be split bt the two sides of the board
const int pinOffset = 22;

// for movement test function:
boolean goForward = false;
const int GO = 50; // enter '2' in serial mon
const int STOP = 49; // enter '1' in serial mon

// for turn function:
float thetaIncrement = 6; // number of degrees per increment while turning

boolean readyToTurn[] = {false, false};
const int RIGHT = 0;
const int LEFT = 1;

const int TURN_LEFT = 44; // comma
const int TURN_RIGHT = 46; // period
boolean turnRight = false;
boolean turnLeft = false;

const float liftAngle = 30; //degrees to lift knee without solving
/////////END GLOBAL VARIABLES/////////////

void setup() {
  Serial.begin(19200);
  Serial.print("Starting...\n");
  // Attach all servos to a pin and put them at start pos:
  for (int i = 0; i < 18; i++) {
    servos[i].attach(i + pinOffset);
    // I believe servo.h uses 0-180, not -90-90:
  }

  pixy.init();
  while (!Serial) {}

  // serialMoveTest Instructions:
  //Serial.println("Enter '2' to start movement, '1' to stop it.");
}

void loop() {
  setReadyStance(); // gets legs ready to move
  // Hexy looks for blocks, rotates to center largest target block in Pixy view,
  // takes steps towards or away from it, then repeats looking for blocks
  // resource: http://www.cmucam.org/projects/cmucam5/wiki/LEGO_Chase_Demo
  // http://www.cmucam.org/projects/cmucam5/wiki/PID_LEGO_Block
  while (true) {
    // look for blocks
    blocks = pixy.getBlocks(); // this function returns number of blocks (objects) detected, starting from 1
    static int i = 0; // only created an initialized the first time loop() is called

    // If the block we want is in view, rotate Hexy to face it, then take x steps towards or away from it
    if (blocks) {
      lastEventTime = millis(); // record time block is detected
      i++;
      int blockSize = 0;
      int blockNum;
      for (j = 0; j < blocks; j++) {
        if (pixy.blocks[j].signature == target) {
          // if multiple blocks detected in target signature, choose largest object to follow
          w = pixy.blocks[j].width;
          h = pixy.blocks[j].height;
          // make the blockSize square based on largest dimension
          //          if (w >= h) {
          //            h = w;
          //          }
          //          else {
          //            w = h;
          //          }
          // make the blockSize square based on width dimension, assuming width is more accurate
          int jsize = w * w;
          if (jsize >= blockSize) {
            blockSize = jsize;
            blockNum = j;
          }
        }
      }
      dist = blockDistance(w); // can send this information to ROS
      x = pixy.blocks[blockNum].x;
      y = pixy.blocks[blockNum].y;
      int xError = xCenter - x;
      int yError = yCenter - y;
      int sizeError = maxSize - blockSize;
      if (i % 50 == 0) {
        pixy.blocks[blockNum].print();
        Serial.print(i); // for debugging
        Serial.print("\t");
        Serial.print(lastEventTime);
        Serial.print("\n");
        if (xError >= xBand) {
          //rotateCCW(xError); // rotate by xError to position object in center of Pixy's view
          turnToThe(LEFT);
          Serial.print("Hexy TURN LEFT\n");
        }
        else if (xError < -xBand) {
          //rotateCW(xError);
          turnToThe(RIGHT);
          Serial.print("Hexy TURN RIGHT\n");
        }
        else {
          Serial.print("Hexy NO TURN\n");
        }
        if (w <= 75) {
          //walkForward(sizeError);
          moveForward();
          Serial.print("Hexy walk FORWARD\n");
        }
//        else if (w > 100) {
//          //walkBackward(sizeError);
//          Serial.print("Hexy walk BACKWARD\n");
//        }
        else {
          Serial.print("Hexy NO walk\n");
        }
      }
    }
    // if no blocks detected, rotate randomly to scan for blocks
    else if (millis() - lastEventTime > 5000) {
      long randNum = random(100);
      //i++;
      //if (i%50==0) {
      if (randNum <= 49) {
        turnToThe(LEFT);
        Serial.print("Hexy search LEFT\n");
        Serial.print(millis());
        lastEventTime = millis();
      }
      else {
        turnToThe(RIGHT);
        Serial.print("Hexy search RIGHT\n");
        Serial.print(millis());
        lastEventTime = millis();
      }
      // }
    }
  }
}

///////////////////////////// GAIT FUNCTIONS /////////////////////////////////
void moveForward() {

  if (tripodLifted[tripod1]) {
    //swings tripod1's legs while tripod2 powers
    if (!fullStep[tripod1] || !fullStep[tripod2]) {
      swingLegs(tripod1);
      powerLegs(tripod2);
    }
    else {
      dropLegs(tripod1);
      fullStep[tripod1] = false;
      liftLegs(tripod2);
      fullStep[tripod2] = false;
    }
  }
  else if (tripodLifted[tripod2]) {
    // vice versa
    if (!fullStep[tripod1] || !fullStep[tripod2]) {
      swingLegs(tripod2);
      powerLegs(tripod1);
    }
    else {
      dropLegs(tripod2);
      fullStep[tripod2] = false;
      liftLegs(tripod1);
      fullStep[tripod1] = false;
    }
  }
  else { //all feet are touching ground, so it lifts a tripod
    if (footPos[tripod1] > footPos[tripod2]) {
      liftLegs(tripod2);
      fullStep[tripod2] = false;
    }
    else {
      liftLegs(tripod1);
      fullStep[tripod1] = false;
    }
  }
}

////////////////////////////////////////////////////////////////////
void liftLegs(int tripod) {
  solveLegs(yLift, z[tripod], tripod);
  moveLegs(tripod);
  tripodLifted[tripod] = true;
  delay(dropDelay); // wait
}
//////////////////////////////////////////////////////////////////
void dropLegs(int tripod) {
  solveLegs(yStand, z[tripod], tripod);
  moveLegs(tripod);
  tripodLifted[tripod] = false;
  delay(dropDelay); // wait
}
//////////////////////////////////////////////////////////////////
void swingLegs(int tripod) {

  if (footPos[tripod] <= maxStep) {
    z[tripod] += stepIncrement;
    Serial.println(z[tripod]);
    solveLegs(yLift, z[tripod], tripod);
    moveLegs(tripod);
  }
  else {
    fullStep[tripod] = true;
  }
}

///////////////////////////////////////////////////////////////////
void powerLegs(int tripod) {

  if (footPos[tripod] >= minStep) {
    z[tripod] -= stepIncrement;
    Serial.println(z[tripod]);
    solveLegs(yStand, z[tripod], tripod);
    moveLegs(tripod);
  }
  else {
    fullStep[tripod] = true;
  }
}

//////////////////////////////////////////////////////////////////
// This puts all feet on the ground, tripod 1 in forward position,
//  tripod2 in rear position.
void setReadyStance(){
  if(tripodLifted[tripod1]){
    solveLegs(yLift, maxStep, tripod1);
    moveLegs(tripod1);
    delay(dropDelay);
    solveLegs(yStand, maxStep, tripod1);
    moveLegs(tripod1);
    delay(dropDelay);
    solveLegs(yLift, 0, tripod2);
    moveLegs(tripod2);
    delay(dropDelay);
    solveLegs(yStand, 0, tripod2);
    moveLegs(tripod2);
    delay(dropDelay);
  }
  else if(tripodLifted[tripod2]){
    solveLegs(yLift, 0, tripod2);
    moveLegs(tripod2);
    delay(dropDelay);
    solveLegs(yStand, 0, tripod2);
    moveLegs(tripod2);
    delay(dropDelay);
    solveLegs(yLift, maxStep, tripod1);
    moveLegs(tripod1);
    delay(dropDelay);
    solveLegs(yStand, maxStep, tripod1);
    moveLegs(tripod1);
    delay(dropDelay);
  }
  else{
    solveLegs(yLift, maxStep, tripod1);
    moveLegs(tripod1);
    delay(dropDelay);
    solveLegs(yStand, maxStep, tripod1);
    moveLegs(tripod1);
    delay(dropDelay);
    solveLegs(yLift, 0, tripod2);
    moveLegs(tripod2);
    delay(dropDelay);
    solveLegs(yStand, 0, tripod2);
    moveLegs(tripod2);
    delay(dropDelay);
  }
  
//  int i;
//  for(i=0;i<18;i++){
//    Serial.print("The servoAngles for "); Serial.print(i+7); Serial.print(" is: ");
//    Serial.println(servoAngles[i]);
//  }
  Serial.print("setReadyStance complete\n");
  
}

/////////////////////////////////////////////////////////////////
void moveLegs(int tripod) {
  if (tripod == tripod1) {
    Serial.println("moving tripod1");
    changeAngles(RF);
    changeAngles(LM);
    changeAngles(RB);
  }
  else {
    Serial.println("moving tripod2");
    changeAngles(LF);
    changeAngles(RM);
    changeAngles(LB);
  }
  footPos[tripod] = solveFootZ(tripod);
  Serial.println(footPos[tripod]);
  delay(incrementDelay); // wait
  Serial.println("moveLegs complete");
}
/////////////////////////////////////////////////////////////////
void solveLegs(float y, float z, int tripod) {
//  if(displayTime){ //doens't seeem used????
    //int startTime = millis();
//  }
  if (tripod == tripod1) {
    solveJoints(xF, y, z, RF);
    solveJoints(xM, y, z - maxStep / 2, LM);
    solveJoints(xF, y, z - maxStep, RB);
  }
  else {
    solveJoints(xF, y, z, LF);
    solveJoints(xM, y, z - maxStep / 2, RM);
    solveJoints(xF, y, z - maxStep, LB);
  }
  Serial.print("solveLegs for tripod #: ");
  Serial.println(tripod);
}
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void changeAngles(int leg) {
  // this will change the angles of all joints for the given leg to the angles in the given array.
  int hip = legIndex(leg, HIP);
  int knee = legIndex(leg, KNEE);
  int ankle = legIndex(leg, ANKLE);
  // 4/27 - the following code was replaced to use servoSelect, which was added at bottom
  if ( leg == LF || leg == LM || leg == LB ) {
    servoSelect(hip + pinOffset, -servoAngles[hip]);
    servoSelect(knee + pinOffset, servoAngles[knee]);
    servoSelect(ankle + pinOffset, -servoAngles[ankle]);
  }
  else {
    servoSelect(hip + pinOffset, servoAngles[hip]);
    servoSelect(knee + pinOffset, servoAngles[knee]);
    servoSelect(ankle + pinOffset, -servoAngles[ankle]);
  }
}

///////////////////////////////////////////////////////////////////
// This will solve joint angles, given x, y, and z in mm, relative to the coxa-body joint.
// Also pass the appropriate joint array: solveJoints( x, y , z, appropriateJointArray ).
// I'll have it return the angles in degrees.
void solveJoints(float x, float y, float z,  int leg) {
  // Declare vars:
  int hip = legIndex(leg, HIP);
  int knee = legIndex(leg, KNEE);
  int ankle = legIndex(leg, ANKLE);
  float L, l, H, t, f, h, tc, alpha, beta, theta;
  // Let's do trig:
  L = abs(sqrt(pow(x, 2) + pow(z, 2))); l = L - COXA;
  H = abs(sqrt(pow(l, 2) + pow(y, 2)));
  t = acos( (pow(FEMUR, 2) + pow(H, 2) - pow(TIBIA, 2)) / (2 * FEMUR * H) );
  f = acos( (pow(TIBIA, 2) + pow(H, 2) - pow(FEMUR, 2)) / (2 * TIBIA * H) );
  h = pi - t - f;
  tc = acos( y / H );
  // Result in radians:
  alpha = pi / 2 - ( t + tc );
  beta = pi - h;
  theta = atan( z / x );
  // Result in degrees:
  servoAngles[hip] = theta * 180 / pi + calibOff[hip];
  servoAngles[knee] = alpha * 180 / pi + calibOff[knee];
  servoAngles[ankle] = beta * 180 / pi + calibOff[ankle];
  //  Serial.println("Solve joints, the servoAngles are: ");
  //  Serial.print("Hip: "); Serial.println(servoAngles[hip]);
  //  Serial.print("Knee: "); Serial.println(servoAngles[knee]);
  //  Serial.print("Ankle: "); Serial.println(servoAngles[ankle]);
}

/////////////////////////////////////////////////////////////////////////////////////
// This solves for z foot position, given joint angles. Useful so the robot knows where it is.
float solveFootZ( int tripod ) {
  // Declare vars:
  float L, y, x, z;
  int leg;
  if (tripod == tripod1) {
    leg = RF;
  }
  else {
    leg = LF;
  }
  int hip = legIndex(leg, HIP);
  int knee = legIndex(leg, KNEE);
  int ankle = legIndex(leg, ANKLE);
  // Convert angles to radians:
  float alpha = (servoAngles[knee] - calibOff[knee]) * pi / 180;
  float beta = (servoAngles[ankle] - calibOff[ankle]) * pi / 180;
  float theta = (servoAngles[hip] - calibOff[hip]) * pi / 180;

  L = COXA + FEMUR * cos(alpha) + TIBIA * cos(alpha + beta);
  y = FEMUR * sin(alpha) + TIBIA * sin(alpha + beta);
  x = L * cos(theta);
  z = L * sin(theta);

  return (z);
}

//////////////////////////////////////////////////
// This converts an angle to ms for the servos
short a2ms(float inAngle) { // change inAngle from short to float 4/28 jg
  if (inAngle > 90) {
    inAngle = 90;
  }
  else if (inAngle < -90) {
    inAngle = -90;
  }
  short MS = inAngle * 1000 / 90 + 1500;
  return MS;
}

/////////////////////////////////////////////////
// This gets input from serial
// '0' is 48, '1' is 49, '2' is 50, look up ascii tables for the rest.
int getVal() {
  int input;
  if (Serial.available() > 0) {
    input = Serial.read();
    //Serial.println(input);
  }
  else {
    input = -1;
  }
  return (input);
}

////////////////////////////////////////////////////
// This function is to test forward movement.
// Enter '2' to move forward, '1' to stop.
void serialMoveTest() {
  int val = getVal();
  if (val == GO) {
    goForward = true;
    Serial.println("GO");
  }

  while (goForward) {
    moveForward();
    val = getVal();
    if (val == STOP) {
      goForward = false;
      Serial.println("STOP");
    }
  }
}

//////////////////////////////////////////////////////
// This outputs the correct array index for the given leg and joint.
// leg constants might need to be altered based on the wiring of the hexy.
int legIndex(int leg, int joint) {
  return (leg * LEG + joint);
}

/////////////////////////////////////////////////////////
// Selects the servo to change, taken from testServo code
void servoSelect( int selectedServo, float inputAngle ) {
  changeServo(selectedServo, a2ms(inputAngle));
}

///////////////////////////////////////////////////////////////////////////
// New function for using servo.h instead of servotor:
void changeServo(int joint, int uS) {
  servos[joint-pinOffset].writeMicroseconds(uS);
}
///////////////////////////TURN FUNCTIONS/////////////////
// This does some turning:
void turnToThe(int turnDirection) {
  // Makes sure legs are coordinated to turn
  if (readyToTurn[turnDirection]) {
    // when tripod 1 is lifted, tripod 2 feet touching
    if (tripodLifted[tripod1]) {
      if (!fullStep[tripod1] || !fullStep[tripod2]) { // if one or the other tripod isnt done moving, keep moving
        swingLegsRot(tripod1, turnDirection);
        powerLegsRot(tripod2, turnDirection);
      }
      else { //both tripods done, so finish the stride
        justDrop(tripod1);
        justLift(tripod2);
        fullStep[tripod1] = false;
        fullStep[tripod2] = false;
      }
    }
    // when tripod 2 is lifted, tripod 1 feet touching
    else if (tripodLifted[tripod2]) {
      if (!fullStep[tripod1] || !fullStep[tripod2]) {
        swingLegsRot(tripod2, turnDirection);
        powerLegsRot(tripod1, turnDirection);
      }
      // both tripods done moving
      else {
        justDrop(tripod2);
        justLift(tripod1);
        fullStep[tripod1] = false;
        fullStep[tripod2] = false;
      }
    }
    else { // all feet on ground, so pick some up
      justLift(tripod1);
      fullStep[tripod1] = false;
    }
  }
  // if legs ren't ready to turn, get them ready:
  else {
    prepareToTurn(turnDirection);
  }
}

///////////////////////////////////////////////////////////////////////////
//This resets legs to prepare for turn
void prepareToTurn(int turnDirection) {
  Serial.print("Preparing to turn  "); Serial.println(turnDirection);
  if (tripodLifted[tripod1]) {
    plantFeetTurn(tripod1, turnDirection);
    justLift(tripod2);
  }
  else if (tripodLifted[tripod2]) {
    plantFeetTurn(tripod2, turnDirection);
    justLift(tripod2);
  }
  else {
    justLift(tripod1);
    plantFeetTurn(tripod1, turnDirection);
    justLift(tripod2);
  }
  Serial.println("Preparation complete");
  readyToTurn[turnDirection] = true;
}

///////////////////////////////////////////////////////////////////////////
// This just lifts legs without solving
void justLift(int tripod) {
  Serial.print("Lifting tripod "); Serial.println(tripod + 1);
  int front; int middle; int back;
  if (tripod == tripod1) {
    front = legIndex(RF, KNEE); middle = legIndex(LM, KNEE); back = legIndex(RB, KNEE);
  }
  else {
    front = legIndex(LF, KNEE); middle = legIndex(RM, KNEE); back = legIndex(LB, KNEE);
  }
  servoAngles[front] = servoAngles[front] - liftAngle;
  servoAngles[middle] = servoAngles[middle] - liftAngle;
  servoAngles[back] = servoAngles[back] - liftAngle;
  moveLegs(tripod);
  tripodLifted[tripod] = true;
  delay(dropDelay);
}

////////////////////////////////////////////////////////////////////////////
// This drops legs without solving
void justDrop(int tripod) {
  Serial.print("Dropping tripod "); Serial.println(tripod + 1);
  int front; int middle; int back;
  if (tripod == tripod1) {
    front = legIndex(RF, KNEE); middle = legIndex(LM, KNEE); back = legIndex(RB, KNEE);
  }
  else {
    front = legIndex(LF, KNEE); middle = legIndex(RM, KNEE); back = legIndex(LB, KNEE);
  }
  servoAngles[front] = START_POS_KNEE;
  servoAngles[middle] = START_POS_KNEE;
  servoAngles[back] = START_POS_KNEE;
  servoAngles[front + 1] = START_POS_ANKLE;
  servoAngles[middle + 1] = START_POS_ANKLE;
  servoAngles[back + 1] = START_POS_ANKLE;
  moveLegs(tripod);
  tripodLifted[tripod] = false;
  delay(dropDelay);
}
////////////////////////////////////////////////////////////////////////////
// This moves lifted legs and plants them to prepare to turn
void plantFeetTurn(int tripod, int turn) {
  swingLegsRot(tripod, turn);
  justDrop(tripod);
  fullStep[tripod] = false;
}

///////////////////////////////////////////////////////////////////////////
// This moves lifted legs into position to plant before turning
void swingLegsRot(int tripod, int turn) {
  Serial.print("Swinging tripod "); Serial.print(tripod + 1); Serial.print(" to the "); Serial.println(turn);
  int front; int middle; int back;
  if (tripod == tripod1) { // Tripod 1
    front = legIndex(RF, HIP); middle = legIndex(LM, HIP); back = legIndex(RB, HIP);
    if (turn == RIGHT) { //prep for right turn
      servoAngles[front] = -bodyAngle + minAngle;
      servoAngles[middle] = maxMiddleAngle;
      servoAngles[back] = 0;
    }
    else { //prep for left turn
      servoAngles[front] = 0;
      servoAngles[middle] = -maxMiddleAngle;
      servoAngles[back] = bodyAngle - minAngle;
    }
  }
  else { // Tripod 2
    front = legIndex(LF, HIP); middle = legIndex(RM, HIP); back = legIndex(LB, HIP);
    if (turn == RIGHT) { //prep for right turn
      servoAngles[front] = 0;
      servoAngles[middle] = -maxMiddleAngle;
      servoAngles[back] = bodyAngle - minAngle;
    }
    else { //prep for left turn
      servoAngles[front] = -bodyAngle + minAngle;
      servoAngles[middle] = maxMiddleAngle;
      servoAngles[back] = 0;
    }
  }
  moveLegs(tripod);
  fullStep[tripod] = true;
//  delay(dropDelay);
//  restLegs(tripod);
}


////////////////////////////////////////////////////////////////////////
// This moves the legs touching the ground while turning
void powerLegsRot(int tripod, int turnDirection) {
  int front; int middle; int back;
  int dirMod = turnModifier(turnDirection);// -1 is left, 1 is right
  int podMod = turnModifier(tripod);
  // Sets servo indices based on which tripod we're moving:
  if (tripod == tripod1) {
    front = legIndex(RF, HIP); middle = legIndex(LM, HIP); back = legIndex(RB, HIP);
  }
  else {
    front = legIndex(LF, HIP); middle = legIndex(RM, HIP); back = legIndex(LB, HIP);
  }
  // Only continues to move if it hasn't yet completed a full step:
  if (!fullStep[tripod]) {
    servoAngles[front] += thetaIncrement * dirMod * podMod;
    servoAngles[middle] -= thetaIncrement * dirMod * podMod;
    servoAngles[back] += thetaIncrement * dirMod * podMod;
    // Conditions for completing a step:
    if (tripod == tripod1) {
      // Tripod 1 turning right:
      if (turnDirection == RIGHT) {
        if (servoAngles[front] >= 0) {
          fullStep[tripod1] = true;
          Serial.print("Step Taken by tripod "); Serial.println(tripod + 1);
          Serial.print(" to the  "); Serial.println(turnDirection);
        }
      }
      // Tripod 1 turning left:
      else {
        if (servoAngles[front] <= -bodyAngle + minAngle) {
          fullStep[tripod1] = true;
          Serial.print("Step Taken by tripod "); Serial.println(tripod + 1);
          Serial.print(" to the  "); Serial.println(turnDirection);
        }
      }
    }
    if (tripod == tripod2) {
      // Tripod 2 turning right:
      if (turnDirection == RIGHT) {
        if (servoAngles[front] <= -bodyAngle + minAngle) {
          fullStep[tripod2] = true;
          Serial.print("Step Taken by tripod "); Serial.println(tripod + 1);
          Serial.print(" to the  "); Serial.println(turnDirection);
        }
      }
      // Tripod 2 turning Left:
      else {
        if (servoAngles[front] >= 0) {
          fullStep[tripod2] = true;
          Serial.print("Step Taken by tripod "); Serial.println(tripod + 1);
          Serial.print(" to the  "); Serial.println(turnDirection);
        }
      }
    }
    moveLegs(tripod);
  }
}

///////////////////////////////////////////////////////////////////////////
// This outputs -1 for left, 1 for right
// And       -1 for tripod2, 1 for tripod1
int turnModifier(int turnDirection) {
  int mod = -1 * ((turnDirection + 1) * 2 - 3);
  return (mod);
}

/////////////////////////////////////////////////////////////////////////////
//IMPORTED FUNCTION (exact)
//  void restLegs(int tripod) {
//    if (tripod == tripod1) {
//      int servosToRest[] = {3, 4, 5, 9, 10, 11, 15, 16, 17};
//      Serial.println("resting tripod1");
//      for (int i = 0; i < 9; i++) {
//        changeServo(servosToRest[i] + pinOffset, -1);
//      }
//    }
//    else {
//      int servosToRest[] = {0, 1, 2, 6, 7, 8, 12, 13, 14};
//      Serial.println("resting tripod2");
//      for (int i = 0; i < 9; i++) {
//        changeServo(servosToRest[i] + pinOffset, -1);
//      }
//    }
//  
//    //footPos[tripod] = solveFootZ(tripod);
//    //hexy.delay_ms(2000); // wait 2s
//  }


////////////////////////// COMPUTER VISION FUNCTIONS ////////////////
/* blockDistance() returns the distance Hexy is away from object
  If the object is always a certain size, scaling it
  allows the robot to calculate distance.
*/
float blockDistance(int width)
{
  // w = width of object, seems more accurate/consistent than height
  // experimentally determined distance / width relationship
  float d = -0.0238 * width + 2.1301;
  return d; // in feet
}

//// pixy2hexy() transforms the Pixy object coordinates to Hexy coordinates
//int pixy2hexy(int pixycoord[], int n)
//{
//  // pixycoord[] is an 1x2 array with x and y coordinates
//  // n is the size of the array, should be 2
//   float transform[4][4] = {
//    { cos(30), -sin(30), 0, 5 },
//    { sin(30), cos(30), 0, 4 },
//    { 0, 0, 1, 0 },
//    { 0, 0, 0, 1 }
//  }; // calculate this transformation matrix - this is arbitrary, need to fix
//  Matrix<2,1> P(pixycoord);
//  Matrix<4,4> T(transform);
//  Matrix<2,1> hexycoord;
//  Multiply(P,T,hexycoord);
//  return hexycoord;
//}
//
