/* 
 *  This is an outline for making the bot move forward, mainly for the 
 *  bot to decide when to lift or drop legs and when to move which tripod,
 *  
 *  All code for controlling the servos has been omitted until we can test this 
 *  and figure out which way we'll do it. I am more familiar with servo.h, but
 *  it may be easier if we can use servotor.h
 *  
 *  /////////////////////////////////////////////////////////////////////////////////
 *  Update 4/16/07:
 *  -I found some reasonable values for the constants we needed. 
 *  -Cleaned up some code by changing a few sets of variables to arrays, 
 *   allowing me to eliminate some if and else statements.
 *  -I added an array for storing servo calibration offsets ( calibOff[] )
 *  -I reordered the pins for each servo as follows:
 *      Starting with pin 7 at the HIP of the FRONT LEFT leg,
 *      pins proceed HIP-KNEE-ANKLE on each leg and then
 *      legs LF-LM-LB-RF-RM-RB
 *    Array positions are ordered the same, but start at position 0.
 *    
 * //////////////////////////////////////////////////////////////////////////////////// 
 *  Update 4/13/17:
 *  I added servo control conde using the servotor library.
 *  It should be ready to test forward movement using the serial monitor on 19200 baud.
 *  Enter '2' to move forward, then '1' to stop. Hopefully it works.
 *  --Or actually I probably need to change a lot of the constants before this is ready. 
 *    A lot of them are just placeholders until I calculate reasonable values.
 *    Specifically, the following constants need to be calculated and/or calibrated:
 *      maxStep, minStep, stepIncrement, yLift, yStand, xF, xM, and maybe a few more.
 *  
 * /////////////////////////////////////////////////////////////////////////////////////
 *  Update 4/22/2017
 *  I fixed the code for each servo's calibration and orientation. I should now give
 *    the correct hip angle for each leg. Calibration offsets have not yet been entered.
 *  Added delays so the servos can keep up with the program.
 *  Added a default ready stance function.
 *    
 */

#include "Servotor32.h" // call the servotor32 Library
Servotor32 hexy; // create a servotor32 object

//Calibration offsets:
const short bodyAngle = 49.22;
// Enter calibration angle for each servo here:
const short calibOff[] = {-bodyAngle,0,0,0,0,0,bodyAngle,0,0,-bodyAngle,0,0,0,0,0,bodyAngle,0,0};

boolean tripodLifted[] = {false,false};
boolean fullStep[] = {false,false};

const int tripod1 = 0;
const int tripod2 = 1;

const short START_POS_HIP = (0);
const short START_POS_KNEE = (45);
const short START_POS_ANKLE = (45);

// Leg measurements in mm:
const short TIBIA = 52.0;
const short FEMUR = 49.0;
const short COXA = 26.0;
//Here's pi:
const short pi = 3.14159265359;

//Here are some contants that might need to be changed:
const short maxStep = 45.92; // this is max z distance foot can move forward
const short minStep = 0; // this is max z distance foot can move backward
const short stepIncrement = 1; // how far each foot moves every loop- lower = smoother but slower
const short yStand = TIBIA + sin(START_POS_KNEE*pi/180); // height of body relative to foot while foot is planted.
const short yLift = yStand -15; // height of body relative to foot while foot is lifted.
const short xF = 39.6128; // lateral distance between foot and body for front feet (also back), while striding
const short xM = 60.6482; // lateral distance between foot and body for middle feet, while striding
// All delays in ms:
const int dropDelay = 500; // How long to wait after telling legs to drop
const int readyDelay = 1000; // How to wait for legs to get into start positoin
const int incrementDelay = 50; // How long to wait each incremental movement

short footPos[] = {maxStep,maxStep}; // z position of front right foot, front left foot
short z[] = {0,0}; // delta z of tripod 1 & 2 

// Initialize joint angle vectors:
short servoAngles[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};
                       
/*
const int SERVO_1 = 0 + pinOffset;
const int SERVO_2 = 1 + pinOffset;
const int SERVO_3 = 2 + pinOffset;
const int SERVO_4 = 3 + pinOffset;
const int SERVO_5 = 4 + pinOffset;
const int SERVO_6 = 5 + pinOffset;
const int SERVO_7 = 6 + pinOffset;
const int SERVO_8 = 7 + pinOffset;
const int SERVO_9 = 8 + pinOffset;
const int SERVO_10 = 9 + pinOffset;
const int SERVO_11 = 10 + pinOffset;
const int SERVO_12 = 11 + pinOffset;
const int SERVO_13 = 12 + pinOffset;
const int SERVO_14 = 13 + pinOffset;
const int SERVO_15 = 14 + pinOffset;
const int SERVO_16 = 15 + pinOffset;
const int SERVO_17 = 16 + pinOffset;
const int SERVO_18 = 17 + pinOffset;
*/

// Or maybe we'll do it this way:
// To get array element number: (leg name)*LEG + (joint name)
//    ex: right front knee => RF*LEG + KNEE = 1*3 + -2 = 1
// To get pin numnber: (leg name)*LEG + (joint name) + pinOffset
//
// We'll probably want to rearrange these according to the best way to hook up the wires,
//  but we should be able to keep the same formula.
const int RF = 4; const int RM = 5; const int RB = 6;
const int LF = 1; const int LM = 2; const int LB = 3;
const int LEG = 3; const int HIP = -3; const int KNEE = -2; const int ANKLE = -1;

// pinOffset allows servos to be split bt the two sides of the board
const int pinOffset = 7;

// for movement test function:
boolean goForward = false;
const int GO = 50; // enter '2' in serial mon
const int STOP = 49; // enter '1' in serial mon

void setup() {
  hexy.begin();
  
  Serial.begin(19200);
  while(!Serial){}
  
  // serialMoveTest Instructions:
  Serial.println("Enter '2' to start movement, '1' to stop it.");
}

void loop() {
  setReadyStance(); // gets legs ready to move
  while(true){ // forever loops
    serialMoveTest(); // Test function for forward movement.
  }
}
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void moveForward(){
  
  if(tripodLifted[tripod1]){ 
    //swings tripod1's legs while tripod2 powers
    if(!fullStep[tripod1]||!fullStep[tripod2]){
      swingLegs(tripod1);
      powerLegs(tripod2);
      }
    else{
      dropLegs(tripod1);
      liftLegs(tripod2);
      }
    }
  else if(tripodLifted[tripod2]){ 
    // vice versa
    if(!fullStep[tripod1]||!fullStep[tripod2]){
      swingLegs(tripod2);
      powerLegs(tripod1);
    }
    else{
      dropLegs(tripod2);
      liftLegs(tripod1);
      }
    }
  else{ //all feet are touching ground, so it lifts a tripod
    if(footPos[tripod1] > footPos[tripod2]){
      liftLegs(tripod2);
    }
    else{
      liftLegs(tripod1);
    }
  }
}

////////////////////////////////////////////////////////////////////
void liftLegs(int tripod){
  solveLegs(yLift,z[tripod],tripod);
  moveLegs(tripod);
  tripodLifted[tripod] = true;
  hexy.delay_ms(dropDelay); // wait
}
//////////////////////////////////////////////////////////////////
void dropLegs(int tripod){
  solveLegs(yStand,z[tripod],tripod);
  moveLegs(tripod);
  tripodLifted[tripod] = false;
  hexy.delay_ms(dropDelay); // wait
}
//////////////////////////////////////////////////////////////////
void swingLegs(int tripod){

  if(footPos[tripod]<=maxStep){
    z[tripod] += stepIncrement;
    solveLegs(yLift,z[tripod],tripod);
    moveLegs(tripod);
  }
  else{
    fullStep[tripod] = true;
  }
}

///////////////////////////////////////////////////////////////////
void powerLegs(int tripod){

  if(footPos[tripod]>=minStep){
    z[tripod] -= stepIncrement;
    solveLegs(yStand,z[tripod],tripod);
    moveLegs(tripod);
  }
  else{
    fullStep[tripod] = true;
  }
}

//////////////////////////////////////////////////////////////////
// This puts all feet on the ground, tripod 1 in forward position, 
//  tripod2 in rear position.
void setReadyStance(){
  solveLegs(yStand, 0, tripod2);
  solveLegs(yStand, maxStep, tripod1);
  moveLegs(tripod1);
  moveLegs(tripod2);
  hexy.delay_ms(readyDelay); // wait for legs to get into position
  Serial.print("setReadyStance complete\n");
}

/////////////////////////////////////////////////////////////////
void moveLegs(int tripod){
  if(tripod==tripod1){
    changeAngles(RF);
    changeAngles(LM);
    changeAngles(RB);
  }
  else{
    changeAngles(LF);
    changeAngles(RM);
    changeAngles(LB);
  }
  footPos[tripod] = solveFootZ(tripod);
  hexy.delay_ms(incrementDelay); // wait
  Serial.print("moveLegs for: ");
  Serial.println(tripod);
}
/////////////////////////////////////////////////////////////////
void solveLegs(short y,short z,int tripod){
  if(tripod==tripod1){
    solveJoints(xF,y,z,RF);
    solveJoints(xM,y,z - maxStep/2,LM);
    solveJoints(xF,y,z - maxStep,RB);
  }
  else{
    solveJoints(xF,y,z,LF);
    solveJoints(xM,y,z - maxStep/2,RM);
    solveJoints(xF,y,z - maxStep,LB);
  }
  Serial.print("solveLegs for: ");
  Serial.println(tripod);
}
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void changeAngles(int leg){
  // this will change the angles of all joints for the given leg to the angles in the given array.
  int hip = legIndex(leg,HIP);
  int knee = legIndex(leg,KNEE);
  int ankle = legIndex(leg,ANKLE);
  if( leg == LF || leg == LM || leg == LB ){
    hexy.changeServo(hip + pinOffset, -a2ms(servoAngles[hip]));
    hexy.changeServo(knee + pinOffset, a2ms(servoAngles[knee]));
    hexy.changeServo(ankle + pinOffset, -a2ms(servoAngles[ankle]));
  }
  else{
    hexy.changeServo(hip + pinOffset, a2ms(servoAngles[hip]));
    hexy.changeServo(knee + pinOffset, a2ms(servoAngles[knee]));
    hexy.changeServo(ankle + pinOffset, -a2ms(servoAngles[ankle]));
  }
}

///////////////////////////////////////////////////////////////////
// This will solve joint angles, given x, y, and z in mm, relative to the coxa-body joint.
// Also pass the appropriate joint array: solveJoints( x, y , z, appropriateJointArray ).
// I'll have it return the angles in degrees.
void solveJoints(short x, short y, short z,  int leg){
  // Declare vars:
  int firstJointIndex = legIndex(leg,HIP);
  short L, l, H, t, f, h, tc, alpha, beta, theta;
  // Let's do trig:
  L = abs(sqrt(pow(x,2)+pow(z,2))); l = L - COXA;
  H = abs(sqrt(pow(l,2)+pow(y,2)));
  t = acos( (pow(FEMUR,2) + pow(H,2) - pow(TIBIA,2)) / (2*FEMUR*H) );
  f = acos( (pow(TIBIA,2) + pow(H,2) - pow(FEMUR,2)) / (2*TIBIA*H) );
  h = pi - t - f;
  tc = acos( y / H );
  // Result in radians:
  alpha = pi / 2 - ( t + tc );
  beta = pi - h;
  theta = atan( z / x );
  // Result in degrees:
  servoAngles[firstJointIndex] = theta * 180 / pi + calibOff[firstJointIndex];
  servoAngles[firstJointIndex+1] = alpha * 180 / pi + calibOff[firstJointIndex+1];
  servoAngles[firstJointIndex+2] = beta * 180 / pi + calibOff[firstJointIndex+2];
}

/////////////////////////////////////////////////////////////////////////////////////
// This solves for z foot position, given joint angles. Useful so the robot knows where it is.
short solveFootZ( int tripod ){
  // Declare vars:
  float L, y, x, z;
  int leg;
  if(tripod==tripod1){
    leg = RF;
  }
  else{
    leg = LF;
  }
  
  // Convert angles to radians:
  float alpha = (servoAngles[legIndex(leg,KNEE)] + calibOff[legIndex(leg,KNEE)]) * pi / 180;
  float beta = (servoAngles[legIndex(leg,ANKLE)] + calibOff[legIndex(leg,ANKLE)]) * pi / 180;
  float theta = (servoAngles[legIndex(leg,HIP)] + calibOff[legIndex(leg,HIP)]) * pi / 180;

  L = COXA + FEMUR * cos(alpha) + TIBIA * cos(alpha+beta);
  y = FEMUR * sin(alpha) + TIBIA * sin(alpha+beta);
  x = L * cos(theta);
  z = L * sin(theta);

  return(z);
}

//////////////////////////////////////////////////
// This converts an angle to ms for the servos
short a2ms(short inAngle){
  if(inAngle>90){
    inAngle = 90;
  }
  else if(inAngle<-90){
    inAngle = -90;
  }
  short MS = inAngle*1000/90+1500;
  return MS;
}

/////////////////////////////////////////////////
// This gets input from serial
// '0' is 48, '1' is 49, '2' is 50, look up ascii tables for the rest.
int getVal(){
  int input;
  if (Serial.available() > 0) {
    input = Serial.read();
    Serial.println(input);
  }
  else{
    input = -1;
  }
  return(input);
}

////////////////////////////////////////////////////
// This function is to test forward movement.
// Enter '2' to move forward, '1' to stop.
void serialMoveTest(){
  int val = getVal();
  if(val==GO){
    goForward = true;
    Serial.println("GO");
    delay(5000);
  }

//  while(val==GO){
//    moveForward();
//    val = getVal();
    if(val==STOP){
      goForward = false;
      Serial.println("STOP");
      servoRest();
    }
 // }
}

///////////////////////////////////////////////////////
// This kills all servos.
void servoRest(){
  // kill all servos
  for(int i=0; i<32; i++){
    hexy.changeServo(i,-1);
  }
}

//////////////////////////////////////////////////////
// This outputs the correct array index for the given leg and joint.
// leg constants might need to be altered based on the wiring of the hexy.
int legIndex(int leg, int joint){
  return (leg*LEG + joint);
}

