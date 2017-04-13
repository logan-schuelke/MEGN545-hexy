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
 *  "[" = previous servo           "]" = next servo
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
int selectedServo = 1;

const int Down = 44; // comma
const int Up = 46; // period
const int increment = 10; // increment 10 degrees per 
const int nextServo = 93; // ]
const int prevServo = 91; // [

const int pinOffset = 0;

#include "Servotor32.h" // call the servotor32 Library
Servotor32 hexy; // create a servotor32 object

// pinOffset allows servos to be split bt the two sides of the board
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


void setup() {
  hexy.begin();
  
  Serial.begin(19200);
  while(!Serial){}
  Serial.println("1 for pos1, 2 for pos2, 0 for neutral");
  Serial.println("comma(<) for decrease angle, period(>) for increase angle");
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
//  Serial.println("1 for pos1, 2 for pos2, 0 for neutral");
//  Serial.println("comma(<) for decrease angle, period(>) for increase angle");
//  Serial.println("[ for prev servo, ] for next servo");
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
  }
  else if( val == Pos2 ){
    currentPos = angle2;
  }
  else if( val == neutral ){
    currentPos = angleN;
  }
  else if( val == Up ){
    currentPos += increment;
    if( currentPos > 90 ){ //Makes sure it doesn't go over 180 deg
      currentPos = 90;
    }    
  }
  else if( val == Down ){
    currentPos -= increment;
    if( currentPos < -90 ){ //Makes sure it doesn't go below 0 deg 
      currentPos = -90;
    }    
  }
  else if( val == nextServo ){
    selectedServo += 1;
    if( selectedServo > SERVO_18 ){ //Makes sure max servo is 18
      selectedServo = SERVO_18;
    }
    currentPos = Pos1;
    Serial.print("Selected servo = ");
    Serial.println(selectedServo);
  }
  else if( val == prevServo ){
    selectedServo -= 1;
    if( selectedServo < SERVO_1 ){ // makes sure min servo is 1
      selectedServo = SERVO_1;
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
    Serial.println(input);
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

