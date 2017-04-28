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

void setup() {
  hexy.begin();
  
  Serial.begin(19200);
  while(!Serial){}
  Serial.println("1 for pos1, 2 for pos2, 0 for neutral");
  Serial.println("Use comma(<) to decrease angle, period(>) to increase angle.");
  Serial.println("For more precise adjustment, use 'n' for down, 'm' for up.");
  Serial.println("[ for prev servo, ] for next servo");
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
    testServos();
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
short a2ms(float inAngle){
  short MS = inAngle*1000/90+1500;
  return MS;
}

