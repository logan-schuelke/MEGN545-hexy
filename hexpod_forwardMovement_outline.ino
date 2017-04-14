/* 
 *  This is an outline for making the bot move forward, mainly for the 
 *  bot to decide when to lift or drop legs and when to move which tripod,
 *  
 *  All code for controlling the servos has been omitted until we can test this 
 *  and figure out which way we'll do it. I am more familiar with servo.h, but
 *  it may be easier if we can use servotor.h
 *  
 *  Update 4/13/17:
 *  I added servo control conde using the servotor library.
 *  It should be ready to test forward movement using the serial monitor on 19200 baud.
 *  Enter '2' to move forward, then '1' to stop. Hopefully it works.
 *  --Or actually I probably need to change a lot of the constants before this is ready. 
 *    A lot of them are just placeholders until I calculate reasonable values.
 *    Specifically, the following constants need to be calculated and/or calibrated:
 *      maxStep, minStep, stepIncrement, yLift, yStand, xF, xM, and maybe a few more.
 *  
 */

#include "Servotor32.h" // call the servotor32 Library
Servotor32 hexy; // create a servotor32 object

//We'll need to figure out reasonable values for all these consts:
const short maxStep = 50; // this is max z distance foot can move forward
const short minStep = -10; // this is max z distance foot can move backward
const short stepIncrement = 1; // how far each foot moves every loop- lower = smoother but slower
const short yLift = 10; // height of body relative to foot while foot is lifted.
const short yStand = 20; // height of body relative to foot while foot is planted.
const short xF = 10; // lateral distance between foot and body for front feet (also back), while striding
const short xM = 15; // lateral distance between foot and body for middle feet, while striding

short footPos1 = 0; // position of front right foot
short footPos2 = 0; // position of front left foot
short z1; // delta z of tripod 1
short z2; // delta z of tripod 2

boolean tripod1Lifted;
boolean tripod2Lifted;

const int tripod1 = 1;
const int tripod2 = 2;

const short START_POS_HIP = (0+90);
const short START_POS_KNEE = (30+90);
const short START_POS_ANKLE = (60+90);

// Leg measurements in mm:
const short TIBIA = 52.0;
const short FEMUR = 49.0;
const short COXA = 26.0;
//Here's pi:
const short pi = 3.14159265359;

// Initialize joint angle vectors:
short servoAngles[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                       START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};
// pinOffset allows servos to be split bt the two sides of the board
const int pinOffset = 0;
const int SERVO_1 = 1 + pinOffset;
const int SERVO_2 = 2 + pinOffset;
const int SERVO_3 = 3 + pinOffset;
const int SERVO_4 = 4 + pinOffset;
const int SERVO_5 = 5 + pinOffset;
const int SERVO_6 = 6 + pinOffset;
const int SERVO_7 = 7 + pinOffset;
const int SERVO_8 = 8 + pinOffset;
const int SERVO_9 = 9 + pinOffset;
const int SERVO_10 = 10 + pinOffset;
const int SERVO_11 = 11 + pinOffset;
const int SERVO_12 = 12 + pinOffset;
const int SERVO_13 = 13 + pinOffset;
const int SERVO_14 = 14 + pinOffset;
const int SERVO_15 = 15 + pinOffset;
const int SERVO_16 = 16 + pinOffset;
const int SERVO_17 = 17 + pinOffset;
const int SERVO_18 = 18 + pinOffset;

// Or maybe we'll do it this way:
// To get array element number: (leg name)*LEG + (joint name)
//    ex: right front knee => RF*LEG + KNEE = 1*3 + -2 = 1
// To get pin numnber: (leg name)*LEG + (joint name) + pinOffset
//
// We'll probably want to rearrange these according to the best way to hook up the wires,
//  but we should be able to keep the same formula.
const int RF = 1; const int RM = 2; const int RB = 3;
const int LF = 4; const int LM = 5; const int LB = 6;
const int LEG = 3; const int HIP = -3; const int KNEE = -2; const int ANKLE = -1;

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
  serialMoveTest(); // Test function for forward movement.
}
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
void moveForward(){
  
  if(tripod1Lifted){ //swings tripod1's legs while tripod2 powers
    swingLegs(tripod1);
    powerLegs(tripod2);
    }
  else if(tripod2Lifted){ // vice versa
    swingLegs(tripod2);
    powerLegs(tripod1);
    }
  else{ //all feet are touching ground, so it lifts a tripod
    if(footPos1 > footPos2){
      liftLegs(tripod2);
    }
    else{
      liftLegs(tripod1);
    }
  }
}

////////////////////////////////////////////////////////////////////
void liftLegs(int tripod){
  if(tripod == tripod1){
    solveLegs(yLift,z1,tripod);
    moveLegs(tripod);
    tripod1Lifted = true;
  }
  else{
    solveLegs(yLift,z2,tripod);
    moveLegs(tripod);
    tripod2Lifted = true;
  }
}
//////////////////////////////////////////////////////////////////
void dropLegs(int tripod){
  if(tripod == tripod1){
    solveLegs(yStand,z1,tripod);
    moveLegs(tripod);
    tripod1Lifted = false;
  }
  else{
    solveLegs(yStand,z2,tripod);
    moveLegs(tripod);
    tripod2Lifted = false;
  }
}
//////////////////////////////////////////////////////////////////
void swingLegs(int tripod){
  if(tripod==tripod1){
    if(footPos1<=maxStep){
      z1 += stepIncrement;
      solveLegs(yLift,z1,tripod);
      moveLegs(tripod);
    }
    else{
      dropLegs(tripod);
    }
  }
  else{
    if(footPos2<=maxStep){
      z2 += stepIncrement;
      solveLegs(yLift,z2,tripod);
      moveLegs(tripod);
    }
    else{
      dropLegs(tripod);
    }
  }
}

///////////////////////////////////////////////////////////////////
void powerLegs(int tripod){
  if(tripod==tripod1){
    if(footPos1>=minStep){
      z1 -= stepIncrement;
      solveLegs(yStand,z1,tripod);
      moveLegs(tripod);
    }
    else{
      liftLegs(tripod);
    }
  }
  else{
    if(footPos2>=minStep){
      z2 -= stepIncrement;
      solveLegs(yStand,z2,tripod);
      moveLegs(tripod);
    }
    else{
      liftLegs(tripod);
    }
  }
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
}
/////////////////////////////////////////////////////////////////
void solveLegs(short y,short z,int tripod){
  if(tripod==tripod1){
    solveJoints(xF,y,z,RF);
    solveJoints(xM,y,z,LM);
    solveJoints(xF,y,z,RB);
  }
  else{
    solveJoints(xF,y,z,LF);
    solveJoints(xM,y,z,RM);
    solveJoints(xF,y,z,LB);
  }
}
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void changeAngles(int leg){
  // this will change the angles of all joints for the given leg to the angles in the given array.
  int hip = legIndex(leg,HIP);
  int knee = hip + 1;
  int ankle = hip + 2;
  hexy.changeServo(hip + pinOffset, a2ms(servoAngles[hip]));
  hexy.changeServo(knee + pinOffset, a2ms(servoAngles[knee]));
  hexy.changeServo(ankle + pinOffset, a2ms(servoAngles[ankle]));
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
  servoAngles[firstJointIndex] = theta * 180 / pi;
  servoAngles[firstJointIndex+1] = alpha * 180 / pi;
  servoAngles[firstJointIndex+2] = beta * 180 / pi;
}

//////////////////////////////////////////////////
// This converts an angle to ms for the servos
short a2ms(short inAngle){
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
  }
  while(val==GO){
    moveForward();
    val = getVal();
    if(val==STOP){
      goForward = false;
      Serial.println("STOP");
    }
  }
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

