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
#include <Servo.h>
Servo servos[18];
 
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

//#include "Servotor32.h" // call the servotor32 Library
//Servotor32 hexy; // create a servotor32 object

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
  const int pinOffset = 22;
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// More stuff for turn functions:

  float thetaIncrement = 6; // number of degrees per increment while turning
  
  boolean readyToTurn[] = {false, false};
  const int RIGHT = 0;
  const int LEFT = 1;
  
  boolean tripodLifted[] = {false,false};
  boolean fullStep[] = {false,false};

  // All delays in ms:
  const int dropDelay = 800; // How long to wait after telling legs to drop
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
  // Attach all servos to a pin and put them at start pos:
  for(int i=0;i<18;i++){
    servos[i].attach(i+pinOffset);
    // I believe servo.h uses 0-180, not -90-90:
    //servos[i].write(startPos[i]+90);
  }
  //hexy.begin();
  
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
      //servoRest();
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
      //servoRest();
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
  // Makes sure legs are coordinated to turn
  if(readyToTurn[turnDirection]){
    // when tripod 1 is lifted, tripod 2 feet touching
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
   // when tripod 2 is lifted, tripod 1 feet touching
   else if(tripodLifted[tripod2]){
    if(!fullStep[tripod1]||!fullStep[tripod2]){
      swingLegsRot(tripod2,turnDirection);
      powerLegsRot(tripod1,turnDirection);
    }
    // both tripods done moving
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
  // if legs ren't ready to turn, get them ready:
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
  delay(dropDelay);
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
  delay(dropDelay);
}
////////////////////////////////////////////////////////////////////////////
// This moves lifted legs and plants them to prepare to turn
void plantFeetTurn(int tripod, int turn){
  swingLegsRot(tripod,turn);
  justDrop(tripod);
  fullStep[tripod]=false;
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
  fullStep[tripod]=true;
  //delay(dropDelay);
  //restLegs(tripod);  
}


////////////////////////////////////////////////////////////////////////
// This moves the legs touching the ground while turning
void powerLegsRot(int tripod,int turnDirection){
  int front; int middle; int back;
  int dirMod = turnModifier(turnDirection);// -1 is left, 1 is right
  int podMod = turnModifier(tripod);
  // Sets servo indices based on which tripod we're moving:
  if(tripod==tripod1){
    front = legIndex(RF,HIP); middle = legIndex(LM,HIP); back = legIndex(RB,HIP);    
  }
  else{
    front = legIndex(LF,HIP); middle = legIndex(RM,HIP); back = legIndex(LB,HIP);
  }
  // Only continues to move if it hasn't yet completed a full step:
  if(!fullStep[tripod]){
    servoAngles[front] += thetaIncrement*dirMod*podMod;
    servoAngles[middle] -= thetaIncrement*dirMod*podMod;
    servoAngles[back] += thetaIncrement*dirMod*podMod;
    // Conditions for completing a step:
    if(tripod==tripod1){
      // Tripod 1 turning right:
      if(turnDirection==RIGHT){
        if(servoAngles[front]>=0){
          fullStep[tripod1] = true;
          Serial.print("Step Taken by tripod ");Serial.println(tripod+1);
          Serial.print(" to the  ");Serial.println(turnDirection);
        }
      }
      // Tripod 1 turning left:
      else{
        if(servoAngles[front]<=-bodyAngle+minAngle){
          fullStep[tripod1] = true;
          Serial.print("Step Taken by tripod ");Serial.println(tripod+1);
          Serial.print(" to the  ");Serial.println(turnDirection);
        }
      }
    }
    if(tripod==tripod2){
      // Tripod 2 turning right:
      if(turnDirection==RIGHT){
        if(servoAngles[front]<=-bodyAngle+minAngle){
          fullStep[tripod2] = true;
          Serial.print("Step Taken by tripod ");Serial.println(tripod+1);
          Serial.print(" to the  ");Serial.println(turnDirection);
        }
      }
      // Tripod 2 turning Left:
      else{
        if(servoAngles[front]>=0){
          fullStep[tripod2] = true;
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
      //delay(2000); // wait 2s
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
      changeServo(selectedServo, a2ms(inputAngle));
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
    short a2ms(float inAngle){
      // This converts angles to ms for servotor:
      //short MS = inAngle*1000/90+1500;
      // This coverts angles to ms for servo.h ( i think ):
      short uS = inAngle*1000/90+1500;
      return uS;
    }

///////////////////////////////////////////////////////////////////////////
// New function for using servo.h instead of servotor:
void changeServo(int joint, int uS){
  servos[joint-pinOffset].writeMicroseconds(uS);
}
