/*
 *  Update 5/5/17:
 *  This is test code to get the thing to turn. It's based on the forward movement outline
 *  but skips the math. 
 *  
 *  I also started to limit the legs' movement. I added a const called minAngle.
 *  
 *  So far I got it to start to take a turn step but it doesn't seem to know when to change
 *  tripods and take another step.
 *
 */
// Serial test values:
const int Pos1 = 49; // number 1
const int angle1 = -90; // angle associated with 1
const int Pos2 = 50; // number 2
const int angle2 = 90; // angle associated with 2
const int neutral = 48; //number 0
const int angleN = 0; // angle associated with neutral
int currentPos = 0;
int selectedServo = 7; //Starts with pin7

const int Down = 44; // comma
const int Up = 46; // period
const int increment = 10; // increment 10 degrees per 
const int nextServo = 93; // ]
const int prevServo = 91; // [
const int smallDown = 110; // 'n'
const int smallUp = 109; // 'm'

#include "Servotor32.h" // call the servotor32 Library
Servotor32 hexy; // create a servotor32 object

const int numberOfServos = 18;
const int firstServo = 7;
///////////////////////////////////////////////////////////////////////////////////////////
//Adding functions here from hexapod_forwardMovement_outline.ino
// to see which of those functions work:

  //Calibration offsets:
  const float bodyAngle = 49.22;
  // Enter calibration angle for each servo here:
  const float calibOff[] = {-bodyAngle,0,0,0,0,0,bodyAngle,0,0,-bodyAngle,0,0,0,0,0,bodyAngle,0,0};
  const float minAngle = 10;
  const float maxMiddleAngle = (bodyAngle-minAngle)/2;
  const int tripod1 = 0;
  const int tripod2 = 1;

  const float START_POS_HIP = (0);
  const float START_POS_KNEE = (45); //45
  const float START_POS_ANKLE = (45); //45

  float servoAngles[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};
  
  const float startPos[]={START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};
                         
  const float liftAngle = 30; //degrees to lift knee without solving

  // To get array element number: (leg name)*LEG + (joint name)
  //    ex: right front knee => RF*LEG + KNEE = 1*3 + -2 = 1
  // To get pin numnber: (leg name)*LEG + (joint name) + pinOffset
  const int RF = 4; const int RM = 5; const int RB = 6;
  const int LF = 1; const int LM = 2; const int LB = 3;
  const int LEG = 3; const int HIP = -3; const int KNEE = -2; const int ANKLE = -1;
  // pinOffset allows servos to be split bt the two sides of the board
  const int pinOffset = 7;
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// More stuff for turn functions:

  float thetaIncrement = 5; // number of degrees per increment while turning
  
  boolean readyToTurn[] = {false, false};
  const int RIGHT = 0;
  const int LEFT = 1;
  
  boolean tripodLifted[] = {false,false};
  boolean fullStep[] = {false,false};

  // All delays in ms:
  const int dropDelay = 500; // How long to wait after telling legs to drop
  const int readyDelay = 1000; // How to wait for legs to get into start positoin
  const int incrementDelay = 0; // How long to wait each incremental movement
  // Serial inputs:
  const int TURN_LEFT = 44; // comma
  const int TURN_RIGHT = 46; // period
  const int STOP = 49; // enter '1' in serial mon
  boolean turnRight = false;
  boolean turnLeft = false;
/////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  hexy.begin();
  
  Serial.begin(19200);
  while(!Serial){}

  // This is the serial output instructions for the original servo test, testServos().
  // If you want to print this, change to true.
  if(false){ 
    Serial.println("1 for pos1, 2 for pos2, 0 for neutral");
    Serial.println("Use comma(<) to decrease angle, period(>) to increase angle.");
    Serial.println("For more precise adjustment, use 'n' for down, 'm' for up.");
    Serial.println("[ for prev servo, ] for next servo");
  }

  // This is the serial output instructions for the new servo test, testAllServos().
  if(false){
    Serial.println("Press 1 to set all joints to zero degrees.");
    Serial.println("Press 2 to set hips to 0, knees and ankles to 45.");
  }

  // This is the serial output instructions for the turn test:
  if(true){
    Serial.println("Use comma(<) to turn left, period(>) to turn right.");
    Serial.println("Press 1 to stop.");
  }
}
//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////MAIN_LOOP////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  servoTestLoop(); // Test range of movement of servos, as well as the angle directionality

}
//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////NEW TURN FUNCTIONS FOR TESTING://////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////
// Loop for testing servos:
void servoTestLoop(){
  while(true){ // loops forever
    //testServos(); //tests each servo indivdually. Good for calibration.
    //testAllServos(); //tests some functions imported from main program
    serialTurnTest();
  }
}

////////////////////////////////////////////////////
// This function is to test forward movement.
// Enter '2' to move forward, '1' to stop.
void serialTurnTest(){
  int val = getVal();
  if(val==TURN_RIGHT){
    turnRight = true;
    turnLeft = false; 
    Serial.println("Turn Right");
  }
  else if(val==TURN_LEFT){
    turnRight = false;
    turnLeft = true; 
    Serial.println("Turn Left");
  }
  // While loop for right turns:
  while(turnRight){
    turnToThe(RIGHT);
    val = getVal();
    if(val==STOP){
      turnRight = false;
      Serial.println("STOP");
      servoRest();
    }
    else if(val==TURN_LEFT){
      turnRight = false;
      turnLeft = true;
      Serial.println("Stoppping and turning other way...");
    }
  }
  // While loop for left turns:
  while(turnLeft){
    turnToThe(LEFT);
    val = getVal();
    if(val==STOP){
      turnLeft = false;
      Serial.println("STOP");
      servoRest();
    }
    else if(val==TURN_RIGHT){
      turnLeft = false;
      turnRight = true;
      Serial.println("Stoppping and turning other way...");
    }
  }
}

//////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
// This does some turning:
void turnToThe(int turnDirection){
  if(readyToTurn[turnDirection]){
   if(tripodLifted[tripod1]){
    if(!fullStep[tripod1]||!fullStep[tripod2]){// if one or the other tripod isnt done moving, keep moving
      swingLegsRot(tripod1,turnDirection);
      powerLegsRot(tripod2,turnDirection);
    }
    else{ //both tripods done, so finish the stride
      justDrop(tripod1);
      justLift(tripod2);
      fullStep[tripod1]=false;
      fullStep[tripod2]=false;
    }
   }
   else if(tripodLifted[tripod2]){
    if(!fullStep[tripod1]||!fullStep[tripod2]){
      swingLegsRot(tripod2,turnDirection);
      powerLegsRot(tripod1,turnDirection);
    }
    else{
      justDrop(tripod2);
      justLift(tripod1);
      fullStep[tripod1]=false;
      fullStep[tripod2]=false;
    }
   }
   else{ // all feet on ground, so pick some up
    justLift(tripod1);
    fullStep[tripod1]=false;
   }
  }
  else{
    prepareToTurn(turnDirection);
  }
}

///////////////////////////////////////////////////////////////////////////
//This resets legs to prepare for turn
void prepareToTurn(int turnDirection){
  Serial.print("Preparing to turn  ");Serial.println(turnDirection);
  if(tripodLifted[tripod1]){
    plantFeetTurn(tripod1,turnDirection);
    justLift(tripod2);    
  }
  else if(tripodLifted[tripod2]){
    plantFeetTurn(tripod2,turnDirection);
    justLift(tripod2);
  }
  else{
    justLift(tripod1);
    plantFeetTurn(tripod1,turnDirection);
    justLift(tripod2);
  }
  Serial.println("Preparation complete");
  readyToTurn[turnDirection]=true;
}

///////////////////////////////////////////////////////////////////////////
// This just lifts legs without solving
void justLift(int tripod){
  Serial.print("Lifting tripod ");Serial.println(tripod+1);
  int front; int middle; int back;
  if(tripod==tripod1){
    front = legIndex(RF,KNEE); middle = legIndex(LM,KNEE); back = legIndex(RB,KNEE);    
  }
  else{
    front = legIndex(LF,KNEE); middle = legIndex(RM,KNEE); back = legIndex(LB,KNEE);
  }
  servoAngles[front] = servoAngles[front] - liftAngle;
  servoAngles[middle] = servoAngles[middle] - liftAngle;
  servoAngles[back] = servoAngles[back] - liftAngle;
  moveLegs(tripod);
  tripodLifted[tripod]=true;
  hexy.delay_ms(dropDelay);
}

////////////////////////////////////////////////////////////////////////////
// This drops legs without solving
void justDrop(int tripod){
  Serial.print("Dropping tripod ");Serial.println(tripod+1);
  int front; int middle; int back;
  if(tripod==tripod1){
    front = legIndex(RF,KNEE); middle = legIndex(LM,KNEE); back = legIndex(RB,KNEE);    
  }
  else{
    front = legIndex(LF,KNEE); middle = legIndex(RM,KNEE); back = legIndex(LB,KNEE);
  }
  servoAngles[front] = START_POS_KNEE;
  servoAngles[middle] = START_POS_KNEE;
  servoAngles[back] = START_POS_KNEE;
  servoAngles[front+1] = START_POS_ANKLE;
  servoAngles[middle+1] = START_POS_ANKLE;
  servoAngles[back+1] = START_POS_ANKLE;
  moveLegs(tripod);
  tripodLifted[tripod]=false;
  hexy.delay_ms(dropDelay);
}
////////////////////////////////////////////////////////////////////////////
// This moves lifted legs and plants them to prepare to turn
void plantFeetTurn(int tripod, int turn){
  swingLegsRot(tripod,turn);
  justDrop(tripod);
}

///////////////////////////////////////////////////////////////////////////
// This moves lifted legs into position to plant before turning
void swingLegsRot(int tripod, int turn){
  Serial.print("Swinging tripod ");Serial.print(tripod+1);Serial.print(" to the ");Serial.println(turn);
  int front; int middle; int back;
  if(tripod==tripod1){// Tripod 1
    front = legIndex(RF,HIP); middle = legIndex(LM,HIP); back = legIndex(RB,HIP);
    if(turn==RIGHT){//prep for right turn
      servoAngles[front] = -bodyAngle+minAngle;
      servoAngles[middle] = maxMiddleAngle;
      servoAngles[back] = 0;
    }
    else{//prep for left turn
      servoAngles[front] = 0;
      servoAngles[middle] = -maxMiddleAngle;
      servoAngles[back] = bodyAngle-minAngle;
    }
  }
  else{// Tripod 2
    front = legIndex(LF,HIP); middle = legIndex(RM,HIP); back = legIndex(LB,HIP);
    if(turn==RIGHT){//prep for right turn
      servoAngles[front] = 0;
      servoAngles[middle] = -maxMiddleAngle;
      servoAngles[back] = bodyAngle-minAngle;
    }
    else{//prep for left turn
      servoAngles[front] = -bodyAngle+minAngle;
      servoAngles[middle] = maxMiddleAngle;
      servoAngles[back] = 0;
    }
  }
  moveLegs(tripod);
  hexy.delay_ms(dropDelay);
}


////////////////////////////////////////////////////////////////////////
// This moves the legs touching the ground while turning
void powerLegsRot(int tripod,int turnDirection){
  int front; int middle; int back;
  int dirMod = turnModifier(turnDirection);// -1 is left, 1 is right
  int podMod = turnModifier(tripod);
  if(tripod==tripod1){
    front = legIndex(RF,HIP); middle = legIndex(LM,HIP); back = legIndex(RB,HIP);    
  }
  else{
    front = legIndex(LF,HIP); middle = legIndex(RM,HIP); back = legIndex(LB,HIP);
  }
  if(!fullStep[tripod]){
    servoAngles[front] += thetaIncrement*dirMod*podMod;
    servoAngles[middle] -= thetaIncrement*dirMod*podMod;
    servoAngles[back] += thetaIncrement*dirMod*podMod;
    if(tripod==tripod1){
      if(turnDirection==RIGHT){
        if(servoAngles[front]>=0){
          fullStep[tripod1] = true;
          Serial.print("Step Taken by tripod ");Serial.println(tripod+1);
          Serial.print(" to the  ");Serial.println(turnDirection);
        }
      }
      else{
        if(servoAngles[front]<=-bodyAngle+minAngle){
          fullStep[tripod1] = true;
          Serial.print("Step Taken by tripod ");Serial.println(tripod+1);
          Serial.print(" to the  ");Serial.println(turnDirection);
        }
      }
    }
    if(tripod==tripod2){
      if(turnDirection==RIGHT){
        if(servoAngles[front]<=-bodyAngle+minAngle){
          fullStep[tripod1] = true;
          Serial.print("Step Taken by tripod ");Serial.println(tripod+1);
          Serial.print(" to the  ");Serial.println(turnDirection);
        }
      }
      else{
        if(servoAngles[front]>=0){
          fullStep[tripod1] = true;
          Serial.print("Step Taken by tripod ");Serial.println(tripod+1);
          Serial.print(" to the  ");Serial.println(turnDirection);
        }
      }
    }
    moveLegs(tripod);
  }
}

///////////////////////////////////////////////////////////////////////////
// This outputs -1 for left, 1 for right
// And       -1 for tripod2, 1 for tripod1
int turnModifier(int turnDirection){
  int mod = -1*((turnDirection+1)*2-3);
  return(mod);  
}

///////////////////////////////////////////////////////////////////////////////
////////////////////OLD FUNCTIONS STILL IN USE:////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////
    // This kills all servos.
    void servoRest(){
      // kill all servos
      for(int i=0; i<32; i++){
        hexy.changeServo(i,-1);
      }
    }
    
    /////////////////////////////////////////////////////////////////////////////
    //IMPORTED FUNCTION (exact)
    void moveLegs(int tripod){
      if(tripod==tripod1){
        Serial.println("moving tripod1");
        changeAngles(RF);
        changeAngles(LM);
        changeAngles(RB);
      }
      else{
        Serial.println("moving tripod2");
        changeAngles(LF);
        changeAngles(RM);
        changeAngles(LB);
      }
      //footPos[tripod] = solveFootZ(tripod);
      //hexy.delay_ms(2000); // wait 2s
    }
    /////////////////////////////////////////////////////////////////////////////
    //IMPORTED FUNCTION (exact)
    void changeAngles(int leg){
      // this will change the angles of all joints for the given leg to the angles in the given array.
      int hip = legIndex(leg,HIP);
      int knee = legIndex(leg,KNEE);
      int ankle = legIndex(leg,ANKLE);
    
      // Here's some serial feedback:
//      Serial.print("changing angle of servo #");
//      Serial.print(hip+pinOffset); Serial.print(" to ");
//      Serial.print(servoAngles[hip]); Serial.println(" degrees.");
//      Serial.print("changing angle of servo #");
//      Serial.print(knee+pinOffset); Serial.print(" to ");
//      Serial.print(servoAngles[knee]); Serial.println(" degrees.");
//      Serial.print("changing angle of servo #");
//      Serial.print(ankle+pinOffset); Serial.print(" to ");
//      Serial.print(servoAngles[ankle]); Serial.println(" degrees.");
      
    if( leg == LF || leg == LM || leg == LB ){
        servoSelect(hip + pinOffset, -servoAngles[hip]);
        servoSelect(knee + pinOffset, servoAngles[knee]);
        servoSelect(ankle + pinOffset, -servoAngles[ankle]);
      }
      else{
        servoSelect(hip + pinOffset, servoAngles[hip]);
        servoSelect(knee + pinOffset, servoAngles[knee]);
        servoSelect(ankle + pinOffset, -servoAngles[ankle]);
      }
    }
    ////////////////////////////////////////////////////////////////////////////
    //IMPORTED FUNCTION (exact)
    // This outputs the correct array index for the given leg and joint.
    int legIndex(int leg, int joint){
      return (leg*LEG + joint);
    }
    
    /////////////////////////////////////////////////
    // This is to select servo for testing.
    // We could probably use this later too.
    void servoSelect( int selectedServo, int inputAngle ){
      hexy.changeServo(selectedServo, a2ms(inputAngle));
    }
    /////////////////////////////////////////////////
    // This gets input from serial
    int getVal(){
      int input;
      if (Serial.available() > 0) {
        input = Serial.read();
        //Serial.println(input);
      }
      else{
        input = -1;
      }
      return(input);
    }
    
    //////////////////////////////////////////////////
    // This converts an angle to ms for the servos
    float a2ms(float inAngle){
      float MS = inAngle*1000/90+1500;
      return MS;
    }
    


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/////////////////////////OLD FUNCTONS///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////
//// This is to test servos:
//void testServos(){
//  int val = getVal();
//  if( val == Pos1 ){
//    currentPos = angle1;
//    Serial.print("Servo angle = ");
//    Serial.println(currentPos);
//  }
//  else if( val == Pos2 ){
//    currentPos = angle2;
//    Serial.print("Servo angle = ");
//    Serial.println(currentPos);
//  }
//  else if( val == neutral ){
//    currentPos = angleN;
//    Serial.print("Servo angle = ");
//    Serial.println(currentPos);
//  }
//  else if( val == Up ){
//    currentPos += increment;
//    if( currentPos > 90 ){ //Makes sure it doesn't go over 180 deg
//      currentPos = 90;
//    }
//    Serial.print("Servo angle = ");
//    Serial.println(currentPos);    
//  }
//  else if( val == Down ){
//    currentPos -= increment;
//    if( currentPos < -90 ){ //Makes sure it doesn't go below 0 deg 
//      currentPos = -90;
//    }    
//    Serial.print("Servo angle = ");
//    Serial.println(currentPos);
//  }
//  else if( val == smallDown ){
//    currentPos -= 1;
//    if( currentPos < -90 ){ //Makes sure it doesn't go below 0 deg 
//      currentPos = -90;
//    }    
//    Serial.print("Servo angle = ");
//    Serial.println(currentPos);
//  }
//  else if( val == smallUp ){
//    currentPos += 1;
//    if( currentPos < -90 ){ //Makes sure it doesn't go below 0 deg 
//      currentPos = -90;
//    }    
//    Serial.print("Servo angle = ");
//    Serial.println(currentPos);
//  }
//  else if( val == nextServo ){
//    selectedServo += 1;
//    if( selectedServo > numberOfServos+firstServo ){ //Makes sure max servo is 18
//      selectedServo = numberOfServos+firstServo-1;
//    }
//    currentPos = Pos1;
//    Serial.print("Selected servo = ");
//    Serial.println(selectedServo);
//  }
//  else if( val == prevServo ){
//    selectedServo -= 1;
//    if( selectedServo < firstServo ){ // makes sure min servo is 1
//      selectedServo = firstServo;
//    }
//    currentPos = Pos1;
//    Serial.print("Selected servo = ");
//    Serial.println(selectedServo);
//  }
//  servoSelect( selectedServo, currentPos );
//}
//
//
///////////////////////////////////////////////////////////////////////////////
//////////////////New Functions for testing////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////////
////NEW FUNCTION
//// Test routine for imported functions:
//void testAllServos(){
//  int val = getVal();
//  if(val==Pos1){
//    Serial.println("Setting all joints to zero degrees...");
//    setAnglesToZero();
//    moveLegs(tripod1);
//    moveLegs(tripod2);
//    Serial.println("...Done");
//  }
//  else if(val==Pos2){
//    Serial.println("Setting all joints to start position...");
//    setAnglesToStart();
//    moveLegs(tripod1);
//    moveLegs(tripod2);
//    Serial.println("...Done");
//  }
//}
///////////////////////////////////////////////////////////////////////////////
////NEW FUNCTION
//// Quickly sets all angles in the array to zero.
//void setAnglesToZero(){
//  for(int i=0;i<18;i++){
//    servoAngles[i] = 0;
//  }
//}
///////////////////////////////////////////////////////////////////////////////
////NEW FUNCTION
//// Sets all angles to default position.
//void setAnglesToStart(){
//  for(int i=0;i<18;i++){
//    servoAngles[i] = startPos[i];
//  }
//}

