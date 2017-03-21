/*
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
const int angle1 = 0; // angle associated with 1
const int Pos2 = 50; // number 2
const int angle2 = 180; // angle associated with 2
const int neutral = 48; //number 0
const int angleN = 90; // angle associated with neutral

const int Down = 44; // comma
const int Up = 46; // period
const int increment = 10; // increment 10 dgrees per 
const int nextServo = 93; // ]
const int prevServo = 91; // [

#include <Servo.h>
// I dont know which pins go where, so i just labeled them 1-18
#define SERVO_1 1
#define SERVO_2 2
#define SERVO_3 3
#define SERVO_4 4
#define SERVO_5 5
#define SERVO_6 6
#define SERVO_7 7
#define SERVO_8 8
#define SERVO_9 9
#define SERVO_10 10
#define SERVO_11 11
#define SERVO_12 12
#define SERVO_13 13
#define SERVO_14 14
#define SERVO_15 15
#define SERVO_16 16
#define SERVO_17 17
#define SERVO_18 18

Servo FrHip; Servo FrKnee; Servo FrAnkle;
Servo FlHip; Servo FlKnee; Servo FlAnkle;
Servo BrHip; Servo BrKnee; Servo BrAnkle;
Servo BlHip; Servo BlKnee; Servo BlAnkle;
Servo MrHip; Servo MrKnee; Servo MrAnkle;
Servo MlHip; Servo MlKnee; Servo MlAnkle;
void setup() {
  FrHip.attach(SERVO_1); FrKnee.attach(SERVO_2); FrAnkle.attach(SERVO_3);
  MrHip.attach(SERVO_4); MrKnee.attach(SERVO_5); MrAnkle.attach(SERVO_6);
  BrHip.attach(SERVO_7); BrKnee.attach(SERVO_8); BrAnkle.attach(SERVO_9);
  FlHip.attach(SERVO_10); FlKnee.attach(SERVO_11); FlAnkle.attach(SERVO_12);
  MlHip.attach(SERVO_13); MlKnee.attach(SERVO_14); MlAnkle.attach(SERVO_15);
  BlHip.attach(SERVO_16); BlKnee.attach(SERVO_17); BlAnkle.attach(SERVO_18);

  FrHip.write(angle1); FrKnee.write(angle1); FrAnkle.write(angle1);
  MrHip.write(angle1); MrKnee.write(angle1); MrAnkle.write(angle1);
  BrHip.write(angle1); BrKnee.write(angle1); BrAnkle.write(angle1);
  FlHip.write(angle1); FlKnee.write(angle1); FlAnkle.write(angle1);
  MlHip.write(angle1); MlKnee.write(angle1); MlAnkle.write(angle1);
  BlHip.write(angle1); BlKnee.write(angle1); BlAnkle.write(angle1);

  Serial.begin(9600);
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
  Serial.println("1 for pos1, 2 for pos2, 0 for neutral");
  Serial.println("comma(<) for decrease angle, period(>) for increase angle");
  Serial.println("[ for prev servo, ] for next servo");
  while(true){ // loops forever
    testServos(SERVO_1);
  }
}
/////////////////////////////////////////////////
// This is to test servos:
void testServos( int selectedServo ){
  int val = getVal();
  int currentPos;
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
    if( currentPos > 180 ){ //Makes sure it doesn't go over 180 deg
      currentPos = 180;
    }    
  }
  else if( val == Down ){
    currentPos -= increment;
    if( currentPos < 0 ){ //Makes sure it doesn't go below 0 deg 
      currentPos = 0;
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
  switch(selectedServo){
    case SERVO_1:
    FrHip.write(inputAngle);
    Serial.println(FrHip.read());
    break;
    case SERVO_2:
    FrKnee.write(inputAngle);
    Serial.println(FrKnee.read());
    break;
    case SERVO_3:
    FrAnkle.write(inputAngle);
    Serial.println(FrAnkle.read());
    break;
    case SERVO_4:
    MrHip.write(inputAngle);
    Serial.println(MrHip.read());
    break;
    case SERVO_5:
    MrKnee.write(inputAngle);
    Serial.println(MrKnee.read());
    break;
    case SERVO_6:
    MrAnkle.write(inputAngle);
    Serial.println(MrAnkle.read());
    break;
    case SERVO_7:
    BrHip.write(inputAngle);
    Serial.println(BrHip.read());
    break;
    case SERVO_8:
    BrKnee.write(inputAngle);
    Serial.println(BrKnee.read());
    break;
    case SERVO_9:
    BrAnkle.write(inputAngle);
    Serial.println(BrAnkle.read());
    break;
    case SERVO_10:
    FlHip.write(inputAngle);
    Serial.println(FlHip.read());
    break;
    case SERVO_11:
    FlKnee.write(inputAngle);
    Serial.println(FlKnee.read());
    break;
    case SERVO_12:
    FlAnkle.write(inputAngle);
    Serial.println(FlAnkle.read());
    break;
    case SERVO_13:
    MlHip.write(inputAngle);
    Serial.println(MlHip.read());
    break;
    case SERVO_14:
    MlKnee.write(inputAngle);
    Serial.println(MlKnee.read());
    break;
    case SERVO_15:
    MlAnkle.write(inputAngle);
    Serial.println(MlAnkle.read());
    break;
    case SERVO_16:
    BlHip.write(inputAngle);
    Serial.println(BlHip.read());
    break;
    case SERVO_17:
    BlKnee.write(inputAngle);
    Serial.println(BlKnee.read());
    break;
    case SERVO_18:
    BlAnkle.write(inputAngle);
    Serial.println(BlAnkle.read());
    break;
  }
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
