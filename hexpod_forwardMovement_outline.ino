/* 
 *  This is an outline for making the bot move forward, mainly for the 
 *  bot to decide when to lift or drop legs and when to move which tripod,
 *  
 *  All code for controlling the servos has been omitted until we can test this 
 *  and figure out which way we'll do it. I am more familiar with servo.h, but
 *  it may be easier if we can use servotor.h
 */

//We'll need to figure out reasonable values for all these consts:
const float maxStep = 50; // this is max z distance foot can move forward
const float minStep = -10; // this is max z distance foot can move backward
const float stepIncrement = 1; // how far each foot moves every loop- lower = smoother but slower
const float yLift = 20;
const float yStand = 10;
const float xF = 10;
const float xM = 15;

float footPos1 = 0; // position of front right foot
float footPos2 = 0; // position of front left foot
float z1;
float z2;

boolean tripod1Lifted;
boolean tripod2Lifted;

const int tripod1 = 1;
const int tripod2 = 2;
const int FR = 1; const int MR = 2; const int BR = 3;
const int FL = 4; const int ML = 5; const int BL = 6;

const float START_POS_HIP = (0+90);
const float START_POS_KNEE = (30+90);
const float START_POS_ANKLE = (60+90);

// Leg measurements in mm:
const float TIBIA = 52.0;
const float FEMUR = 49.0;
const float COXA = 26.0;
//Here's pi:
const float pi = 3.14159265359;

// Initialize joint angle vectors:
float FRjoints[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};
float MRjoints[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};
float BRjoints[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};
float FLjoints[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};
float MLjoints[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};
float BLjoints[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

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
    changeAngles(FR,FRjoints);
    changeAngles(ML,MLjoints);
    changeAngles(BR,BRjoints);
  }
  else{
    changeAngles(FL,FLjoints);
    changeAngles(MR,MRjoints);
    changeAngles(BL,BLjoints);
  }
}
/////////////////////////////////////////////////////////////////
void solveLegs(float y,float z,int tripod){
  if(tripod==tripod1){
    solveJoints(xF,y,z,FRjoints);
    solveJoints(xM,y,z,MLjoints);
    solveJoints(xF,y,z,BRjoints);
  }
  else{
    solveJoints(xF,y,z,FLjoints);
    solveJoints(xM,y,z,MRjoints);
    solveJoints(xF,y,z,BLjoints);
  }
}
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
void changeAngles(int leg, float jointArray[]){
  // this will change the angles of all joints for the given leg to the angles in the given array.
}

///////////////////////////////////////////////////////////////////
// This will solve joint angles, given x, y, and z in mm, relative to the coxa-body joint.
// Also pass the appropriate joint array: solveJoints( x, y , z, appropriateJointArray ).
// I'll have it return the angles in degrees.
void solveJoints(float x, float y, float z,  float jointArray[]){
  // Declare vars:
  float L, l, H, t, f, h, tc, alpha, beta, theta;
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
  jointArray[0] = theta * 180 / pi;
  jointArray[1] = alpha * 180 / pi;
  jointArray[2] = beta * 180 / pi;
}
