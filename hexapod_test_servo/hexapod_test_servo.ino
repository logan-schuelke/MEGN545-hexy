

// To change to servo.h, added function changeServo(). This replaces hexy.changeServo().
// Altered a2ms slightly. I think servo.h sends ms from 1000-2000 instead of 500-2500.
// Added include servo.h and created an array of servo objects.
// Made a for loop in setup that attaches the servos to pins and starts them at 
//  their start position.

// I think that's all we need really.
// Oh, and I found all hexy.delay_ms and changed to delay.

#include <Servo.h>
Servo servos[18];



/*
 * This is now updated to use the Servotor.h library rather than Servo.h. 
 * When using serial monitor, ensure you have the right baud rate (19200).
 * 
 *This is code to test servo position and range of motion. From this we should be able to get the offsets
 *for each straight joint. Also we may be able to calibrate the nonlinearity of the servos this way.
 *I'm not really sure how to do that though.
 *
 *Enter commands into the Serial Monitor (make sure its at 9600):
 *  "1" = Pos1 (0 degrees)    "2" = Pos2 (180 degrees)    0"" = neutral (90 degrees)
 *  "," = subtract 10 degrees      "." = add 10 degrees
 *  "n" = subtract 1 degree        "m" = add 1 degree
 *  "[" = previous servo           "]" = next servo
 * 
 *  
 * Update 4/17/17:  
 *  I added code to adjust servos by smaller increments and changed the pinout to match
 *  the pinout for the forward_movement_outline.
 *
 * Update 4/26/17:
 *  I added some functions from our main movement program so we can test them, along
 *  with their associated global variables and constants.
 *  
 *  moveLegs(), legIndex, and changeAngles()
 *
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

//#include "Servotor32.h" // call the servotor32 Library
//Servotor32 hexy; // create a servotor32 object



const int numberOfServos = 18;
const int firstServo = 7;
///////////////////////////////////////////////////////////////////////////////////////////
//Adding functions here from hexapod_forwardMovement_outline.ino
// to see which of those functions work:

  //Calibration offsets:
  const short bodyAngle = 49.22;
  // Enter calibration angle for each servo here:
  const short calibOff[] = {-bodyAngle,0,0,0,0,0,bodyAngle,0,0,-bodyAngle,0,0,0,0,0,bodyAngle,0,0};

  const int tripod1 = 0;
  const int tripod2 = 1;

  const short START_POS_HIP = (0);
  const short START_POS_KNEE = (0); //45
  const short START_POS_ANKLE = (0); //45

  short servoAngles[] = {START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};
  
  const short startPos[]={START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE,
                         START_POS_HIP, START_POS_KNEE, START_POS_ANKLE};

  // To get array element number: (leg name)*LEG + (joint name)
  //    ex: right front knee => RF*LEG + KNEE = 1*3 + -2 = 1
  // To get pin numnber: (leg name)*LEG + (joint name) + pinOffset
  const int RF = 4; const int RM = 5; const int RB = 6;
  const int LF = 1; const int LM = 2; const int LB = 3;
  const int LEG = 3; const int HIP = -3; const int KNEE = -2; const int ANKLE = -1;
  // pinOffset allows servos to be split bt the two sides of the board
  const int pinOffset = 7;
//////////////////////////////////////////////////////////////////////////////////////////

// Attach all servos to a pin and put them at start pos:
void setup() {
  for(int i=0;i<18;i++){
    servos[i].attach(i+pinOffset);
    // I believe servo.h uses 0-180, not -90-90:
    servos[i].write(startPos[i]+90);
  }



  
  //hexy.begin();
  
  Serial.begin(19200);
  while(!Serial){}

  // This is the serial output instructions for the original servo test, testServos().
  // If you want to print this, change to true.
  if(true){ 
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
}
//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////MAIN_LOOP////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  servoTestLoop(); // Test range of movement of servos, as well as the angle directionality

}
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////
// Loop for testing servos:
void servoTestLoop(){
  while(true){ // loops forever
    testServos(); //tests each servo indivdually. Good for calibration.
    //testAllServos(); //tests some functions imported from main program
  }
}
/////////////////////////////////////////////////
// This is to test servos:
void testServos(){
  int val = getVal();
  if( val == Pos1 ){
    currentPos = angle1;
    Serial.print("Servo angle = ");
    Serial.println(currentPos);
  }
  else if( val == Pos2 ){
    currentPos = angle2;
    Serial.print("Servo angle = ");
    Serial.println(currentPos);
  }
  else if( val == neutral ){
    currentPos = angleN;
    Serial.print("Servo angle = ");
    Serial.println(currentPos);
  }
  else if( val == Up ){
    currentPos += increment;
    if( currentPos > 90 ){ //Makes sure it doesn't go over 180 deg
      currentPos = 90;
    }
    Serial.print("Servo angle = ");
    Serial.println(currentPos);    
  }
  else if( val == Down ){
    currentPos -= increment;
    if( currentPos < -90 ){ //Makes sure it doesn't go below 0 deg 
      currentPos = -90;
    }    
    Serial.print("Servo angle = ");
    Serial.println(currentPos);
  }
  else if( val == smallDown ){
    currentPos -= 1;
    if( currentPos < -90 ){ //Makes sure it doesn't go below 0 deg 
      currentPos = -90;
    }    
    Serial.print("Servo angle = ");
    Serial.println(currentPos);
  }
  else if( val == smallUp ){
    currentPos += 1;
    if( currentPos < -90 ){ //Makes sure it doesn't go below 0 deg 
      currentPos = -90;
    }    
    Serial.print("Servo angle = ");
    Serial.println(currentPos);
  }
  else if( val == nextServo ){
    selectedServo += 1;
    if( selectedServo > numberOfServos+firstServo ){ //Makes sure max servo is 18
      selectedServo = numberOfServos+firstServo;
    }
    currentPos = Pos1;
    Serial.print("Selected servo = ");
    Serial.println(selectedServo);
  }
  else if( val == prevServo ){
    selectedServo -= 1;
    if( selectedServo < firstServo ){ // makes sure min servo is 1
      selectedServo = firstServo;
    }
    currentPos = Pos1;
    Serial.print("Selected servo = ");
    Serial.println(selectedServo);
  }
  servoSelect( selectedServo, currentPos );
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
// This converts an angle to us for the servos
short a2ms(float inAngle){
  // This converts angles to ms for servotor:
  //short MS = inAngle*1000/90+1500;
  // This coverts angles to ms for servo.h ( i think ):
  short uS = inAngle*500/90+1500;
  return uS;
}

/////////////////////////////////////////////////////////////////////////////
////////////////New Functions for testing////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//NEW FUNCTION
// Test routine for imported functions:
void testAllServos(){
  int val = getVal();
  if(val==Pos1){
    Serial.println("Setting all joints to sero degrees...");
    setAnglesToZero();
    moveLegs();
    Serial.println("...Done");
  }
  else if(val==Pos2){
    Serial.println("Setting all joints to start position...");
    setAnglesToStart();
    moveLegs();
    Serial.println("...Done");
  }
}
/////////////////////////////////////////////////////////////////////////////
//NEW FUNCTION
// Quickly sets all angles in the array to zero.
void setAnglesToZero(){
  for(int i=0;i<18;i++){
    servoAngles[i] = 0;
  }
}
/////////////////////////////////////////////////////////////////////////////
//NEW FUNCTION
// Sets all angles to default position.
void setAnglesToStart(){
  for(int i=0;i<18;i++){
    servoAngles[i] = startPos[i];
  }
}
/////////////////////////////////////////////////////////////////////////////
//IMPORTED FUNCTION (modified)
// This is mostly the same function as the one from fwdMove_outline.
// It has been modified to move both tripods at once.
void moveLegs(){
  //if(tripod==tripod1){
    Serial.println("moving tripod1");
    changeAngles(RF);
    changeAngles(LM);
    changeAngles(RB);
  //}
  //else{
    Serial.println("moving tripod2");
    changeAngles(LF);
    changeAngles(RM);
    changeAngles(LB);
  //}
  //footPos[tripod] = solveFootZ(tripod);
  delay(2000); // wait 2s
}
/////////////////////////////////////////////////////////////////////////////
//IMPORTED FUNCTION (exact)
void changeAngles(int leg){
  // this will change the angles of all joints for the given leg to the angles in the given array.
  int hip = legIndex(leg,HIP);
  int knee = legIndex(leg,KNEE);
  int ankle = legIndex(leg,ANKLE);

  // Here's some serial feedback:
  Serial.print("changing angle of servo #");
  Serial.print(hip+pinOffset); Serial.print(" to ");
  Serial.print(servoAngles[hip]); Serial.println(" degrees.");
  Serial.print("changing angle of servo #");
  Serial.print(knee+pinOffset); Serial.print(" to ");
  Serial.print(servoAngles[knee]); Serial.println(" degrees.");
  Serial.print("changing angle of servo #");
  Serial.print(ankle+pinOffset); Serial.print(" to ");
  Serial.print(servoAngles[ankle]); Serial.println(" degrees.");
  
  if( leg == LF || leg == LM || leg == LB ){
    changeServo(hip + pinOffset, -a2ms(servoAngles[hip]));
    changeServo(knee + pinOffset, a2ms(servoAngles[knee]));
    changeServo(ankle + pinOffset, -a2ms(servoAngles[ankle]));
  }
  else{
    changeServo(hip + pinOffset, a2ms(servoAngles[hip]));
    changeServo(knee + pinOffset, a2ms(servoAngles[knee]));
    changeServo(ankle + pinOffset, -a2ms(servoAngles[ankle]));
  }
}
////////////////////////////////////////////////////////////////////////////
//IMPORTED FUNCTION (exact)
// This outputs the correct array index for the given leg and joint.
int legIndex(int leg, int joint){
  return (leg*LEG + joint);
}

////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
// New function for using servo.h instead of servotor:
void changeServo(int joint, int uS){
  servos[joint].writeMicroseconds(uS);
}

